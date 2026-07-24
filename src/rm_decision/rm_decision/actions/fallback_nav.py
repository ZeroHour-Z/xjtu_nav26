import math
from typing import List, Optional

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from ..registry import register


@register("NavigateWithFallbackAction")
class NavigateWithFallbackAction(py_trees.behaviour.Behaviour):
	"""带备选目标点的导航动作。

	依次尝试 ``waypoints`` 中的目标点：当前目标点若因底盘卡住
	(``trapped_topic`` 收到 True) 或导航失败/超时而无法抵达，则切换到
	下一个备选点，循环往复。到达任一目标点即返回 SUCCESS。

	典型用途：打符点、巡逻区等有多个可选点位的状态，卡在半路时换点去。
	"""

	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		frame_id: str = "map",
		waypoints: Optional[List[dict]] = None,
		trapped_topic: str = "/chassis_trapped",
		timeout_s: Optional[float] = None,
		arrival_tolerance_m: float = 0.0,
		base_frame: str = "base_link",
		switch_on_trapped: bool = True,
		switch_on_failure: bool = True,
		success_on_arrival: bool = True,
		cancel_on_terminate: bool = True,
		publish_goal_topic: str = "/goal_pose",
	):
		super().__init__(name)
		self.node = node
		self.client = ActionClient(node, NavigateToPose, server_name)
		self.frame_id = str(frame_id)
		self.waypoints = list(waypoints or [])
		self.trapped_topic = str(trapped_topic)
		self.timeout_s = timeout_s
		self.arrival_tolerance_m = max(0.0, float(arrival_tolerance_m))
		self.base_frame = str(base_frame)
		self.switch_on_trapped = bool(switch_on_trapped)
		self.switch_on_failure = bool(switch_on_failure)
		self.success_on_arrival = bool(success_on_arrival)
		self.cancel_on_terminate = bool(cancel_on_terminate)

		self._goal_pub = (
			self.node.create_publisher(PoseStamped, publish_goal_topic, 10)
			if publish_goal_topic
			else None
		)
		# 卡住标志订阅（在构造函数中创建，保证 tick 前已能收到）
		self._trapped = False
		self._trapped_sub = self.node.create_subscription(
			Bool, self.trapped_topic, self._on_trapped, 10
		)

		# TF 用于到点距离判定（可选）
		self._tf_buffer = None
		self._tf_listener = None
		if self.arrival_tolerance_m > 0.0:
			from tf2_ros import Buffer, TransformListener

			self._tf_buffer = Buffer()
			self._tf_listener = TransformListener(self._tf_buffer, self.node)

		self._waypoint_index = 0
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self._goal_rejected = False
		self._start_time = None
		self._goal_generation = 0

	def setup(self, **kwargs) -> None:
		# 不抛异常：与 Nav2 栈同时启动时 action server 可能尚未 active，
		# 改为在发送目标前惰性检查，保证 BT 节点仍能启动。
		# 非阻塞探测：不在 setup 阶段阻塞等待 server（多个动作串行各等几秒会
		# 撑爆 py_trees_ros 的 setup 总超时预算导致节点崩溃）。
		if not self.client.wait_for_server(timeout_sec=0.0):
			self.node.get_logger().warn(
				f"{self.name}: Nav2 NavigateToPose server not ready at setup; "
				f"will wait for it at tick time"
			)

	def initialise(self) -> None:
		self._waypoint_index = 0
		self._reset_goal_state()

	def update(self) -> Status:
		if not self.waypoints:
			self.node.get_logger().warn(f"{self.name}: no waypoints configured")
			return Status.FAILURE

		# 到点判定（距离），成功即结束
		if self.success_on_arrival and self._is_at_waypoint(self.waypoints[self._waypoint_index]):
			self.node.get_logger().info(
				f"{self.name}: reached waypoint={self._waypoint_index} by distance check"
			)
			self._cancel_goal()
			return Status.SUCCESS

		if not self._sent:
			if not self.client.server_is_ready():
				self._start_time = self.node.get_clock().now()
				return Status.RUNNING
			self._send_goal(self.waypoints[self._waypoint_index])
			return Status.RUNNING

		# 卡住 -> 换点
		if self.switch_on_trapped and self._trapped:
			self.node.get_logger().warn(
				f"{self.name}: chassis trapped at waypoint={self._waypoint_index}, switching"
			)
			self._switch_waypoint("trapped")
			return Status.RUNNING

		# 目标被拒 -> 换点
		if self._goal_rejected:
			if self.switch_on_failure:
				self._switch_waypoint("goal rejected")
				return Status.RUNNING
			return Status.FAILURE

		# 超时 -> 换点
		if self.timeout_s is not None:
			elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
			if elapsed > self.timeout_s:
				self._cancel_goal()
				if self.switch_on_failure:
					self._switch_waypoint("goal timeout")
					return Status.RUNNING
				return Status.FAILURE

		# 导航结果
		if self._result_future is not None and self._result_future.done():
			result_stub = self._result_future.result()
			status_code = getattr(result_stub, "status", None)
			if status_code is None and self._goal_handle is not None:
				status_code = getattr(self._goal_handle, "status", None)
			if status_code == 4:  # STATUS_SUCCEEDED
				self.node.get_logger().info(
					f"{self.name}: reached waypoint={self._waypoint_index}"
				)
				return Status.SUCCESS
			else:
				if self.switch_on_failure:
					self._switch_waypoint(f"goal status={status_code}")
					return Status.RUNNING
				return Status.FAILURE

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID:
			self._goal_generation += 1
		if new_status != Status.RUNNING and self.cancel_on_terminate:
			self._cancel_goal()

	def _switch_waypoint(self, reason: str) -> None:
		self._cancel_goal()
		self._waypoint_index = (self._waypoint_index + 1) % len(self.waypoints)
		self.node.get_logger().warn(
			f"{self.name}: {reason}, switching to waypoint={self._waypoint_index}"
		)
		self._reset_goal_state()

	def _send_goal(self, waypoint: dict) -> None:
		pose = self._build_pose(waypoint)
		goal = NavigateToPose.Goal()
		goal.pose = pose
		if self._goal_pub is not None:
			self._goal_pub.publish(pose)
		self._goal_generation += 1
		generation = self._goal_generation
		send_future = self.client.send_goal_async(goal)
		send_future.add_done_callback(
			lambda future, generation=generation: self._on_goal_response(future, generation)
		)
		self._sent = True
		self._start_time = self.node.get_clock().now()
		# 发送新目标后清除卡住标志：electronic control 端会在收到新 /goal_pose 后
		# 复位监测，本地也先清除避免用旧标志立即再次换点。
		self._trapped = False
		self.node.get_logger().info(
			f"{self.name}: sending goal to waypoint={self._waypoint_index} "
			f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
		)

	def _build_pose(self, waypoint: dict) -> PoseStamped:
		pose = PoseStamped()
		pose.header.frame_id = self.frame_id
		pose.header.stamp = self.node.get_clock().now().to_msg()
		pose.pose.position.x = float(waypoint.get("x", 0.0))
		pose.pose.position.y = float(waypoint.get("y", 0.0))
		yaw = float(waypoint.get("yaw", 0.0))
		pose.pose.orientation.z = math.sin(yaw * 0.5)
		pose.pose.orientation.w = math.cos(yaw * 0.5)
		return pose

	def _is_at_waypoint(self, waypoint: dict) -> bool:
		if self.arrival_tolerance_m <= 0.0 or self._tf_buffer is None:
			return False
		from rclpy.time import Time
		from tf2_ros import TransformException

		try:
			transform = self._tf_buffer.lookup_transform(
				self.frame_id, self.base_frame, Time()
			)
		except TransformException:
			return False
		current_x = transform.transform.translation.x
		current_y = transform.transform.translation.y
		goal_x = float(waypoint.get("x", 0.0))
		goal_y = float(waypoint.get("y", 0.0))
		return math.hypot(goal_x - current_x, goal_y - current_y) <= self.arrival_tolerance_m

	def _cancel_goal(self) -> None:
		if self._goal_handle is not None:
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as exc:
				self.node.get_logger().warn(f"{self.name}: cancel goal failed: {exc}")

	def _reset_goal_state(self) -> None:
		self._goal_generation += 1
		self._goal_handle = None
		self._result_future = None
		self._goal_rejected = False
		self._sent = False
		self._start_time = None

	def _on_trapped(self, msg: Bool) -> None:
		self._trapped = bool(msg.data)

	def _on_goal_response(self, future, generation: int):
		goal_handle = future.result()
		if generation != self._goal_generation:
			if getattr(goal_handle, "accepted", False):
				try:
					goal_handle.cancel_goal_async()
				except Exception as exc:
					self.node.get_logger().warn(f"{self.name}: cancel stale goal failed: {exc}")
			return
		self._goal_handle = goal_handle
		if not getattr(self._goal_handle, "accepted", False):
			self._result_future = None
			self._goal_rejected = True
			self.node.get_logger().warn(f"{self.name}: goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async()
