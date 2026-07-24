from typing import Optional
import math

import py_trees
from py_trees.common import Status
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from ..registry import register


@register("ChaseDynamicPointAction")
class ChaseDynamicPointAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		topic: str = "/chase_point",
		server_name: str = "navigate_to_pose",
		cancel_on_terminate: bool = True,
		min_target_delta: float = 0.2,
		min_resend_interval_s: float = 0.3,
		publish_goal_topic: str = "",
		publish_chase_goal_topic: str = "",
	):
		super().__init__(name)
		self.node = node
		self.topic = topic
		self.client = ActionClient(node, NavigateToPose, server_name)
		self.cancel_on_terminate = cancel_on_terminate
		self.min_target_delta = float(min_target_delta)
		self.min_resend_interval_s = float(min_resend_interval_s)
		self._goal_handle = None
		self._result_future = None
		self._last_pose: Optional[PoseStamped] = None
		self._pending_pose: Optional[PoseStamped] = None
		self._has_new_target: bool = False
		self._last_send_time = None
		self._goal_generation = 0
		self._goal_pub = (
			self.node.create_publisher(PoseStamped, publish_goal_topic, 10)
			if publish_goal_topic
			else None
		)
		self._chase_goal_pub = (
			self.node.create_publisher(PoseStamped, publish_chase_goal_topic, 10)
			if publish_chase_goal_topic
			else None
		)
		# 在构造函数中立即创建订阅，确保能收到消息
		self._sub = self.node.create_subscription(PoseStamped, self.topic, self._on_target, 10)
		self.node.get_logger().info(f"ChaseDynamicPointAction: subscribed to {topic}")

	def setup(self, **kwargs) -> None:
		# 订阅已在 __init__ 中创建，这里只需探测 action server。
		# 不抛异常：与 Nav2 栈同时启动时(如 sentry_bringup)，action server
		# 可能尚未 active。改为在 tick 时惰性等待，保证 BT 节点仍能启动。
		# 非阻塞探测：不在 setup 阶段阻塞等待，避免多个动作串行累加撑爆
		# py_trees_ros 的 setup 总超时预算导致节点崩溃。
		if not self.client.wait_for_server(timeout_sec=0.0):
			self.node.get_logger().warn(
				f"{self.name}: Nav2 NavigateToPose server not ready at setup; "
				f"will wait for it at tick time"
			)

	def initialise(self) -> None:
		if self._last_send_time is None:
			self._last_send_time = self.node.get_clock().now()
		# 不重置 _has_new_target！
		# 订阅回调可能在 tick 间隙已经收到新目标，重置会导致目标丢失。
		# 如果 _last_pose 已有值，标记为新目标以触发首次发送。
		if self._pending_pose is not None:
			self._has_new_target = True
		elif self._last_pose is not None:
			self._has_new_target = True

	def update(self) -> Status:
		# Must have a target before we can chase
		if self._last_pose is None and self._pending_pose is None:
			self.node.get_logger().debug("ChaseDynamicPointAction: waiting for target pose...")
			return Status.RUNNING

		# If we got a new target, (re)send goal
		if self._has_new_target:
			# Wait until the action server is available before sending.
			if not self.client.server_is_ready():
				return Status.RUNNING
			now = self.node.get_clock().now()
			elapsed = (now - self._last_send_time).nanoseconds / 1e9 if self._last_send_time is not None else 1e9
			if elapsed < self.min_resend_interval_s:
				return Status.RUNNING
			target_pose = self._pending_pose if self._pending_pose is not None else self._last_pose
			if target_pose is None:
				return Status.RUNNING
			self.node.get_logger().info(
				f"ChaseDynamicPointAction: sending goal to ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})"
			)
			# Cancel previous goal if active
			if self._goal_handle is not None:
				try:
					self._goal_handle.cancel_goal_async()
				except Exception as exc:
					self.node.get_logger().warn(f"Cancel previous chase goal failed: {exc}")
			# Send new goal
			goal_msg = NavigateToPose.Goal()
			goal_msg.pose = target_pose
			if self._goal_pub is not None:
				self._goal_pub.publish(target_pose)
			if self._chase_goal_pub is not None:
				self._chase_goal_pub.publish(target_pose)
			self._goal_generation += 1
			generation = self._goal_generation
			send_future = self.client.send_goal_async(goal_msg)
			send_future.add_done_callback(
				lambda future, generation=generation: self._on_goal_response(future, generation)
			)
			self._last_pose = target_pose
			self._pending_pose = None
			self._last_send_time = now
			self._has_new_target = False
			return Status.RUNNING

		# If current goal finished, we still keep running to accept new targets
		if self._result_future is not None and self._result_future.done():
			# Ignore result status to keep chasing loop alive
			return Status.RUNNING

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status != Status.RUNNING:
			self._goal_generation += 1
		if new_status != Status.RUNNING and self.cancel_on_terminate and self._goal_handle is not None:
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as exc:
				self.node.get_logger().warn(f"Cancel chase goal failed on terminate: {exc}")

	def _on_target(self, pose: PoseStamped) -> None:
		if self._last_pose is None:
			self._pending_pose = pose
			self._has_new_target = True
			return

		dx = pose.pose.position.x - self._last_pose.pose.position.x
		dy = pose.pose.position.y - self._last_pose.pose.position.y
		distance = math.hypot(dx, dy)
		if distance < self.min_target_delta:
			return

		self._pending_pose = pose
		self._has_new_target = True

	def _on_goal_response(self, future, generation: int):
		goal_handle = future.result()
		if generation != self._goal_generation:
			if getattr(goal_handle, 'accepted', False):
				try:
					goal_handle.cancel_goal_async()
				except Exception as exc:
					self.node.get_logger().warn(f"Cancel stale chase goal failed: {exc}")
			return
		self._goal_handle = goal_handle
		if not getattr(self._goal_handle, 'accepted', False):
			self._result_future = None
			# self.node.get_logger().warn("Chase goal rejected by server")
		else:
			# self.node.get_logger().info("Chase goal accepted by server")
			self._result_future = self._goal_handle.get_result_async() 
