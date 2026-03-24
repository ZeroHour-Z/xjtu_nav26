#!/usr/bin/env python3
# coding: utf-8

import os
import time
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2


class PcdPublisherNode(Node):
	def __init__(self) -> None:
		super().__init__('pcd_publisher')
		self.declare_parameter('map', '')
		self.declare_parameter('frame_id', 'map3d')
		self.declare_parameter('rate', 5.0)
		self.declare_parameter('filter_invalid_points', True)
		self.declare_parameter('max_abs_coord', 100000.0)

		transient_local_qos = QoSProfile(depth=1)
		transient_local_qos.reliability = ReliabilityPolicy.RELIABLE
		transient_local_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

		self.pub = self.create_publisher(PointCloud2, '/map3d', transient_local_qos)

		path = self.get_parameter('map').value
		if not path or not os.path.isfile(path):
			self.get_logger().error(f'Invalid PCD path: {path}')
			self.points = np.zeros((0, 3), dtype=np.float32)
		else:
			pcd = o3d.io.read_point_cloud(path)
			self.points = np.asarray(pcd.points, dtype=np.float32)
			self.points = self._sanitize_points(self.points)
			self.get_logger().info(f'Loaded PCD: {path} with {self.points.shape[0]} valid points')

		rate = float(self.get_parameter('rate').value)
		self.timer = self.create_timer(1.0 / max(1e-6, rate), self.on_timer)

	def _sanitize_points(self, points: np.ndarray) -> np.ndarray:
		if points.size == 0:
			return points

		if not bool(self.get_parameter('filter_invalid_points').value):
			return points

		max_abs_coord = float(self.get_parameter('max_abs_coord').value)
		finite_mask = np.isfinite(points).all(axis=1)
		range_mask = (np.abs(points) <= max_abs_coord).all(axis=1)
		keep_mask = finite_mask & range_mask

		removed = int(points.shape[0] - np.count_nonzero(keep_mask))
		if removed > 0:
			removed_non_finite = int(np.count_nonzero(~finite_mask))
			removed_out_of_range = int(np.count_nonzero(finite_mask & ~range_mask))
			self.get_logger().warn(
				f'Filtered invalid map points: removed {removed} '
				f'(non-finite={removed_non_finite}, out-of-range={removed_out_of_range}, '
				f'max_abs_coord={max_abs_coord}), kept {int(np.count_nonzero(keep_mask))}'
			)

		return points[keep_mask]

	def on_timer(self) -> None:
		header = Header()
		header.frame_id = self.get_parameter('frame_id').value
		header.stamp = self.get_clock().now().to_msg()
		msg = pc2.create_cloud_xyz32(header, self.points.tolist())
		self.pub.publish(msg)


def main():
	rclpy.init()
	node = PcdPublisherNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 