import math
from typing import List, Optional

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped

from ..registry import register


@register("NavigateToPoseAction")
class NavigateToPoseAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		goal_pose: Optional[PoseStamped] = None,
		frame_id: str = "map",
		x: float = 0.0,
		y: float = 0.0,
		yaw: float = 0.0,
		timeout_s: Optional[float] = None,
		cancel_on_terminate: bool = True,
		retry_on_failure: bool = False,
		publish_goal_topic: str = "",
	):
		super().__init__(name)
		self.node = node
		self.client = ActionClient(node, NavigateToPose, server_name)
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self._goal_rejected = False
		self.timeout_s = timeout_s
		self.cancel_on_terminate = cancel_on_terminate
		self.retry_on_failure = retry_on_failure
		self._start_time = None
		self._goal_pose = goal_pose
		self._frame_id = frame_id
		self._x = x
		self._y = y
		self._yaw = yaw
		self._goal_generation = 0
		self._goal_pub = (
			self.node.create_publisher(PoseStamped, publish_goal_topic, 10)
			if publish_goal_topic
			else None
		)

	def setup(self, **kwargs) -> None:
		# kwargs may include timeout, node, visitor. Use timeout if provided; otherwise block briefly.
		timeout_sec = float(kwargs.get('timeout', 3.0)) if 'timeout' in kwargs else 3.0
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			raise RuntimeError("Nav2 NavigateToPose action server not available")

	def initialise(self) -> None:
		self._goal_generation += 1
		self._sent = False
		self._goal_handle = None
		self._result_future = None
		self._goal_rejected = False
		self._start_time = self.node.get_clock().now()

	def update(self) -> Status:
		if not self._sent:
			goal_msg = NavigateToPose.Goal()
			if self._goal_pose is None:
				pose = PoseStamped()
				pose.header.frame_id = self._frame_id
				pose.header.stamp = self.node.get_clock().now().to_msg()
				pose.pose.position.x = float(self._x)
				pose.pose.position.y = float(self._y)
				pose.pose.orientation.z = math.sin(self._yaw * 0.5)
				pose.pose.orientation.w = math.cos(self._yaw * 0.5)
				goal_msg.pose = pose
			else:
				goal_msg.pose = self._goal_pose
			if self._goal_pub is not None:
				self._goal_pub.publish(goal_msg.pose)
			self.node.get_logger().info(
				f"{self.name}: sending NavigateToPose goal "
				f"({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})"
			)
			self._goal_generation += 1
			generation = self._goal_generation
			send_future = self.client.send_goal_async(goal_msg)
			send_future.add_done_callback(
				lambda future, generation=generation: self._on_goal_response(future, generation)
			)
			self._sent = True
			return Status.RUNNING

		if self._goal_rejected:
			return Status.FAILURE

		# timeout handling
		if self.timeout_s is not None:
			elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
			if elapsed > self.timeout_s:
				if self._goal_handle is not None:
					try:
						self._goal_handle.cancel_goal_async()
					except Exception as exc:
						self.node.get_logger().warn(f"Cancel goal failed: {exc}")
				return Status.FAILURE

		if self._result_future is not None and self._result_future.done():
			result_stub = self._result_future.result()
			# Prefer result status when available
			status_code = getattr(result_stub, 'status', None)
			if status_code is None and self._goal_handle is not None:
				status_code = getattr(self._goal_handle, 'status', None)
			if status_code == 4:  # STATUS_SUCCEEDED
				return Status.SUCCESS
			else:
				if self.retry_on_failure:
					self.node.get_logger().warn(
						f"NavigateToPose result status={status_code}, retrying same goal"
					)
					self._sent = False
					self._goal_handle = None
					self._result_future = None
					self._start_time = self.node.get_clock().now()
					self._goal_rejected = False
					return Status.RUNNING
				return Status.FAILURE

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID:
			self._goal_generation += 1
		if new_status == Status.INVALID and self.cancel_on_terminate and self._goal_handle is not None:
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as exc:
				self.node.get_logger().warn(f"Cancel goal failed on terminate: {exc}")

	def _on_goal_response(self, future, generation: int):
		goal_handle = future.result()
		if generation != self._goal_generation:
			if getattr(goal_handle, 'accepted', False):
				try:
					goal_handle.cancel_goal_async()
				except Exception as exc:
					self.node.get_logger().warn(f"Cancel stale goal failed: {exc}")
			return
		self._goal_handle = goal_handle
		if not getattr(self._goal_handle, 'accepted', False):
			self._result_future = None
			self._goal_rejected = True
			self.node.get_logger().warn(f"NavigateToPose goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async()


@register("NavigateThroughPosesAction")
class NavigateThroughPosesAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_through_poses",
		frame_id: str = "map",
		poses: Optional[List[dict]] = None,
		timeout_s: Optional[float] = None,
		cancel_on_terminate: bool = True,
	):
		super().__init__(name)
		self.node = node
		self.client = ActionClient(node, NavigateThroughPoses, server_name)
		self._goal_handle = None
		self._result_future = None
		self._sent = False
		self._succeeded = False
		self._goal_rejected = False
		self.timeout_s = timeout_s
		self.cancel_on_terminate = cancel_on_terminate
		self._start_time = None
		self._frame_id = frame_id
		self._poses_raw = poses or []

	def setup(self, **kwargs) -> None:
		timeout_sec = float(kwargs.get('timeout', 3.0)) if 'timeout' in kwargs else 3.0
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			raise RuntimeError("Nav2 NavigateThroughPoses action server not available")

	def initialise(self) -> None:
		if self._succeeded:
			return
		self._sent = False
		self._goal_handle = None
		self._result_future = None
		self._goal_rejected = False
		self._start_time = self.node.get_clock().now()

	def _build_pose_stamped(self, p: dict) -> PoseStamped:
		pose = PoseStamped()
		pose.header.frame_id = self._frame_id
		pose.header.stamp = self.node.get_clock().now().to_msg()
		pose.pose.position.x = float(p.get("x", 0.0))
		pose.pose.position.y = float(p.get("y", 0.0))
		yaw = float(p.get("yaw", 0.0))
		pose.pose.orientation.z = math.sin(yaw * 0.5)
		pose.pose.orientation.w = math.cos(yaw * 0.5)
		return pose

	def update(self) -> Status:
		if self._succeeded:
			return Status.SUCCESS

		if not self._sent:
			goal_msg = NavigateThroughPoses.Goal()
			goal_msg.poses = [self._build_pose_stamped(p) for p in self._poses_raw]
			send_future = self.client.send_goal_async(goal_msg)
			send_future.add_done_callback(self._on_goal_response)
			self._sent = True
			self.node.get_logger().info(
				f"NavigateThroughPoses: sending {len(goal_msg.poses)} waypoints"
			)
			return Status.RUNNING

		if self._goal_rejected:
			return Status.FAILURE

		if self.timeout_s is not None:
			elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
			if elapsed > self.timeout_s:
				if self._goal_handle is not None:
					try:
						self._goal_handle.cancel_goal_async()
					except Exception as exc:
						self.node.get_logger().warn(f"Cancel goal failed: {exc}")
				return Status.FAILURE

		if self._result_future is not None and self._result_future.done():
			result_stub = self._result_future.result()
			status_code = getattr(result_stub, 'status', None)
			if status_code is None and self._goal_handle is not None:
				status_code = getattr(self._goal_handle, 'status', None)
			if status_code == 4:  # STATUS_SUCCEEDED
				self._succeeded = True
				return Status.SUCCESS
			else:
				return Status.FAILURE

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID:
			self._succeeded = False
			if self.cancel_on_terminate and self._goal_handle is not None:
				try:
					self._goal_handle.cancel_goal_async()
				except Exception as exc:
					self.node.get_logger().warn(f"Cancel goal failed on terminate: {exc}")

	def _on_goal_response(self, future):
		self._goal_handle = future.result()
		if not getattr(self._goal_handle, 'accepted', False):
			self._result_future = None
			self._goal_rejected = True
			self.node.get_logger().warn("NavigateThroughPoses goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async()
