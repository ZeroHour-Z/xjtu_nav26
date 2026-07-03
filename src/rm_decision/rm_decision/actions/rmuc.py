import math
from typing import Dict, List, Optional

import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformException, TransformListener

from ..registry import register


@register("RegionPatrolAction")
class RegionPatrolAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		frame_id: str = "map",
		region_param: str = "patrol_region",
		regions: Optional[Dict] = None,
		default_region: Optional[int] = None,
		dwell_s: float = 2.0,
		timeout_s: Optional[float] = None,
		max_retries_per_waypoint: int = 3,
		retry_delay_s: float = 1.0,
		advance_on_failure: bool = True,
		arrival_tolerance_m: float = 0.0,
		base_frame: str = "base_link",
		cancel_on_terminate: bool = True,
		publish_goal_topic: str = "/goal_pose",
	):
		super().__init__(name)
		self.node = node
		self.client = ActionClient(node, NavigateToPose, server_name)
		self.frame_id = frame_id
		self.region_param = region_param.lstrip("/")
		self.regions = self._normalise_regions(regions or {})
		self.default_region = default_region
		self.dwell_s = float(dwell_s)
		self.timeout_s = timeout_s
		self.max_retries_per_waypoint = max(0, int(max_retries_per_waypoint))
		self.retry_delay_s = max(0.0, float(retry_delay_s))
		self.advance_on_failure = bool(advance_on_failure)
		self.arrival_tolerance_m = max(0.0, float(arrival_tolerance_m))
		self.base_frame = str(base_frame)
		self.cancel_on_terminate = bool(cancel_on_terminate)
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self.node)
		self.goal_pub = (
			self.node.create_publisher(PoseStamped, publish_goal_topic, 10)
			if publish_goal_topic
			else None
		)
		self._goal_handle = None
		self._result_future = None
		self._goal_rejected = False
		self._sent = False
		self._active_region = None
		self._waypoint_index = 0
		self._start_time = None
		self._dwell_start = None
		self._retry_count = 0
		self._retry_after = None
		self._goal_generation = 0

	def setup(self, **kwargs) -> None:
		# 不抛异常：与 Nav2 栈同时启动时 action server 可能尚未 active，
		# 改为在发送目标前惰性检查，保证 BT 节点仍能启动。
		timeout_sec = float(kwargs.get("timeout", 3.0)) if "timeout" in kwargs else 3.0
		if not self.node.has_parameter(self.region_param):
			self.node.declare_parameter(self.region_param, 0)
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			self.node.get_logger().warn(
				f"{self.name}: Nav2 NavigateToPose server not ready at setup; "
				f"will wait for it at tick time"
			)

	def initialise(self) -> None:
		self._reset_goal_state()

	def update(self) -> Status:
		region_id = self._read_region_id()
		waypoints = self._waypoints_for_region(region_id)
		if not waypoints:
			self.node.get_logger().warn(f"RegionPatrolAction: no waypoints for patrol_region={region_id}")
			return Status.FAILURE

		if self._active_region != region_id:
			self._cancel_goal()
			self._active_region = region_id
			self._waypoint_index = 0
			self._retry_count = 0
			self._retry_after = None
			self._reset_goal_state()

		if self._dwell_start is not None:
			elapsed = (self.node.get_clock().now() - self._dwell_start).nanoseconds / 1e9
			if elapsed < self.dwell_s:
				return Status.RUNNING
			self._dwell_start = None
			self._advance_waypoint(len(waypoints))

		if self._retry_after is not None:
			if self.node.get_clock().now() < self._retry_after:
				return Status.RUNNING
			self._retry_after = None

		if not self._sent:
			# Wait until the action server is available before sending.
			if not self.client.server_is_ready():
				return Status.RUNNING
			self._send_goal(waypoints[self._waypoint_index])
			return Status.RUNNING

		if self._is_at_waypoint(waypoints[self._waypoint_index]):
			self.node.get_logger().info(
				f"RegionPatrolAction: reached waypoint={self._waypoint_index} by distance check"
			)
			self._cancel_goal()
			self._retry_count = 0
			self._dwell_start = self.node.get_clock().now()
			self._reset_goal_state(keep_dwell=True, keep_retry=True)
			return Status.RUNNING

		if self._goal_rejected:
			self._handle_goal_failure(len(waypoints), "goal rejected")
			return Status.RUNNING

		if self.timeout_s is not None:
			elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
			if elapsed > self.timeout_s:
				self._cancel_goal()
				self._handle_goal_failure(len(waypoints), "goal timeout")
				return Status.RUNNING

		if self._result_future is not None and self._result_future.done():
			result_stub = self._result_future.result()
			status_code = getattr(result_stub, "status", None)
			if status_code is None and self._goal_handle is not None:
				status_code = getattr(self._goal_handle, "status", None)
			if status_code == 4:
				self._retry_count = 0
				self._dwell_start = self.node.get_clock().now()
			else:
				self._handle_goal_failure(len(waypoints), f"goal status={status_code}")
				return Status.RUNNING
			self._reset_goal_state(keep_dwell=True, keep_retry=True)

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID and self.cancel_on_terminate:
			self._cancel_goal()
			self._reset_goal_state()

	def _read_region_id(self) -> int:
		try:
			return int(self.node.get_parameter(self.region_param).value)
		except Exception:
			return int(self.default_region or 0)

	def _waypoints_for_region(self, region_id: int) -> List[dict]:
		if region_id in self.regions:
			return self.regions[region_id]
		if self.default_region is not None and self.default_region in self.regions:
			return self.regions[self.default_region]
		return []

	def _send_goal(self, waypoint: dict) -> None:
		pose = self._build_pose(waypoint)
		goal = NavigateToPose.Goal()
		goal.pose = pose
		if self.goal_pub is not None:
			self.goal_pub.publish(pose)
		self._goal_generation += 1
		generation = self._goal_generation
		send_future = self.client.send_goal_async(goal)
		send_future.add_done_callback(
			lambda future, generation=generation: self._on_goal_response(future, generation)
		)
		self._sent = True
		self._start_time = self.node.get_clock().now()
		self.node.get_logger().info(
			f"RegionPatrolAction: region={self._active_region}, waypoint={self._waypoint_index}, "
			f"goal=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
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
		if self.arrival_tolerance_m <= 0.0:
			return False
		try:
			transform = self.tf_buffer.lookup_transform(
				self.frame_id,
				self.base_frame,
				Time(),
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
				self.node.get_logger().warn(f"RegionPatrolAction: cancel goal failed: {exc}")

	def _reset_goal_state(self, keep_dwell: bool = False, keep_retry: bool = False) -> None:
		self._goal_generation += 1
		self._goal_handle = None
		self._result_future = None
		self._goal_rejected = False
		self._sent = False
		self._start_time = None
		if not keep_dwell:
			self._dwell_start = None
		if not keep_retry:
			self._retry_after = None

	def _handle_goal_failure(self, waypoint_count: int, reason: str) -> None:
		if not self.advance_on_failure or self._retry_count < self.max_retries_per_waypoint:
			self._retry_count += 1
			self._retry_after = self.node.get_clock().now() + Duration(seconds=self.retry_delay_s)
			if self.advance_on_failure:
				retry_text = f"({self._retry_count}/{self.max_retries_per_waypoint})"
			else:
				retry_text = f"(attempt {self._retry_count}, holding current waypoint)"
			self.node.get_logger().warn(
				f"RegionPatrolAction: {reason}, retrying waypoint={self._waypoint_index} {retry_text}"
			)
			self._reset_goal_state(keep_retry=True)
			return

		self.node.get_logger().warn(
			f"RegionPatrolAction: {reason}, switching to next waypoint after "
			f"{self._retry_count} retries"
		)
		self._advance_waypoint(waypoint_count)

	def _advance_waypoint(self, waypoint_count: int) -> None:
		self._waypoint_index = (self._waypoint_index + 1) % waypoint_count
		self._retry_count = 0
		self._retry_after = None
		self._reset_goal_state()

	def _on_goal_response(self, future, generation: int):
		if generation != self._goal_generation:
			return
		self._goal_handle = future.result()
		if not getattr(self._goal_handle, "accepted", False):
			self._result_future = None
			self._goal_rejected = True
			self.node.get_logger().warn("RegionPatrolAction: goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async()

	@staticmethod
	def _normalise_regions(raw: Dict) -> Dict[int, List[dict]]:
		regions: Dict[int, List[dict]] = {}
		for key, waypoints in raw.items():
			regions[int(key)] = list(waypoints or [])
		return regions
