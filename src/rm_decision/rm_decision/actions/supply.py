from typing import Optional

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from ..registry import register


class _SupplyState:
	IDLE = 0
	HEAL_NAV = 1
	HEAL_WAIT = 2
	RELOAD_NAV = 3
	RELOAD_WAIT = 4


@register("SupplyManagerAction")
class SupplyManagerAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		hp_param: str = "hp",
		ammo_param: str = "ammo",
		hp_low: float = 200.0,
		hp_high: float = 400.0,
		ammo_low: float = 30.0,
		ammo_high: float = 99.0,
		frame_id: str = "map",
		heal_x: float = -4.0,
		heal_y: float = -6.0,
		heal_yaw: float = 0.0,
		reload_x: float = 0.0,
		reload_y: float = 0.0,
		reload_yaw: float = 0.0,
		nav_timeout_s: Optional[float] = None,
		cancel_on_terminate: bool = True,
	):
		super().__init__(name)
		self.node = node

		def _dg(key: str, default):
			full = f"supply.{key}"
			try:
				if not self.node.has_parameter(full):
					self.node.declare_parameter(full, default)
				return self.node.get_parameter(full).value
			except Exception:
				return default

		# Read action client configuration first
		server_name = str(_dg("server_name", server_name))
		self.client = ActionClient(node, NavigateToPose, server_name)
		self.cancel_on_terminate = bool(_dg("cancel_on_terminate", cancel_on_terminate))
		_nav_timeout_default = nav_timeout_s if nav_timeout_s is not None else 120.0
		nav_timeout_val = _dg("nav_timeout_s", _nav_timeout_default)
		self.nav_timeout_s = float(nav_timeout_val) if nav_timeout_val is not None else None

		# Threshold and topic parameter names
		self.hp_param = str(_dg("hp_param", hp_param)).lstrip("/")
		self.ammo_param = str(_dg("ammo_param", ammo_param)).lstrip("/")
		self.hp_low = float(_dg("hp_low", 200.1))
		self.hp_high = float(_dg("hp_high", 399.9))
		self.ammo_low = float(_dg("ammo_low", 10.1))
		self.ammo_high = float(_dg("ammo_high", 99.9))

		# Frames and fixed waypoints
		self.frame_id = str(_dg("frame_id", frame_id))
		self.heal_x = float(_dg("heal_x", -4.0))
		self.heal_y = float(_dg("heal_y", -6.0))
		self.heal_yaw = float(_dg("heal_yaw", 0.0))
		self.reload_x = float(_dg("reload_x", -4.0))
		self.reload_y = float(_dg("reload_y", -6.0))
		self.reload_yaw = float(_dg("reload_yaw", 0.0))

		self._state = _SupplyState.IDLE
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self._start_time = None

	def setup(self, **kwargs) -> None:
		# Ensure action server available. Do NOT raise if missing: when launched
		# alongside the Nav2 stack the server may not be active yet; we check
		# server_is_ready() before sending a goal instead.
		# 非阻塞探测（见 NavigateToPoseAction.setup 说明）：不在 setup 阻塞等待 server。
		if not self.client.wait_for_server(timeout_sec=0.0):
			self.node.get_logger().warn(
				f"{self.name}: Nav2 NavigateToPose server not ready at setup; "
				f"will wait for it at tick time"
			)
		# Declare params if absent
		try:
			if not self.node.has_parameter(self.hp_param):
				self.node.declare_parameter(self.hp_param, float(self.hp_high))
			if not self.node.has_parameter(self.ammo_param):
				self.node.declare_parameter(self.ammo_param, float(self.ammo_high))
		except Exception:
			pass

	def initialise(self) -> None:
		# Reset per-run state
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self._start_time = self.node.get_clock().now()
		# When entering, evaluate what to do
		self._state = _SupplyState.IDLE

	def _get_param_float(self, name: str) -> Optional[float]:
		try:
			val = self.node.get_parameter(name).value
			return float(val) if val is not None else None
		except Exception:
			return None

	def _build_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
		pose = PoseStamped()
		pose.header.frame_id = self.frame_id
		pose.header.stamp = self.node.get_clock().now().to_msg()
		pose.pose.position.x = x
		pose.pose.position.y = y
		import math
		pose.pose.orientation.z = math.sin(yaw * 0.5)
		pose.pose.orientation.w = math.cos(yaw * 0.5)
		goal = NavigateToPose.Goal()
		goal.pose = pose
		return goal

	def _send_goal(self, goal: NavigateToPose.Goal) -> None:
		send_future = self.client.send_goal_async(goal)
		send_future.add_done_callback(self._on_goal_response)
		self._sent = True

	def _cancel_goal(self) -> None:
		if self._goal_handle is not None:
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as exc:
				self.node.get_logger().warn(f"Cancel goal failed: {exc}")

	def update(self) -> Status:
		# Evaluate state transitions
		hp = self._get_param_float(self.hp_param)
		ammo = self._get_param_float(self.ammo_param)

		# If sensor values missing, keep running (no decision)
		if hp is None or ammo is None:
			return Status.RUNNING

		if self._state == _SupplyState.IDLE:
			# Wait until the action server is available before deciding/sending.
			if (hp <= self.hp_low or ammo <= self.ammo_low) and not self.client.server_is_ready():
				return Status.RUNNING
			# Decide what to do: healing first, then reloading
			if hp <= self.hp_low:
				goal = self._build_goal(self.heal_x, self.heal_y, self.heal_yaw)
				self._send_goal(goal)
				self._state = _SupplyState.HEAL_NAV
				return Status.RUNNING
			elif ammo <= self.ammo_low:
				goal = self._build_goal(self.reload_x, self.reload_y, self.reload_yaw)
				self._send_goal(goal)
				self._state = _SupplyState.RELOAD_NAV
				return Status.RUNNING
			else:
				return Status.FAILURE

		# Handle navigation progress
		if self._state in (_SupplyState.HEAL_NAV, _SupplyState.RELOAD_NAV):
			# Timeout
			if self.nav_timeout_s is not None:
				elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
				if elapsed > self.nav_timeout_s:
					self._cancel_goal()
					# give up this tick
					return Status.FAILURE
			# Check result
			if self._result_future is not None and self._result_future.done():
				result_stub = self._result_future.result()
				status_code = getattr(result_stub, 'status', None)
				if status_code == 4:  # STATUS_SUCCEEDED
					self._start_time = self.node.get_clock().now()
					self._state = _SupplyState.HEAL_WAIT if self._state == _SupplyState.HEAL_NAV else _SupplyState.RELOAD_WAIT
					return Status.RUNNING
				else:
					# Navigation failed, re-evaluate next tick
					self._state = _SupplyState.IDLE
					return Status.RUNNING
			return Status.RUNNING

		# Handle waiting for thresholds
		if self._state == _SupplyState.HEAL_WAIT:
			if hp >= self.hp_high:
				return Status.SUCCESS
			return Status.RUNNING
		if self._state == _SupplyState.RELOAD_WAIT:
			if ammo >= self.ammo_high:
				return Status.SUCCESS
			return Status.RUNNING

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID and self.cancel_on_terminate:
			self._cancel_goal()

	def _on_goal_response(self, future):
		self._goal_handle = future.result()
		if not getattr(self._goal_handle, 'accepted', False):
			self._result_future = None
			self.node.get_logger().warn(f"Supply goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async() 