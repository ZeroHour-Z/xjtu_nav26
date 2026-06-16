"""
区域检测节点 launch 文件
用于检测机器人是否在特殊区域（如颠簸路段）
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution(
        [FindPackageShare('rm_terrain_analysis'), 'config', 'region_detector.yaml']
    )

    region_detector_node = Node(
        package='rm_terrain_analysis',
        executable='region_detector_node',
        name='region_detector_node',
        output='screen',
        parameters=[config_dir]
    )

    return LaunchDescription([
        region_detector_node,
    ])
