#!/usr/bin/env python3
# coding: utf-8
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from datetime import datetime
from pathlib import Path
import os


def generate_launch_description():
    # Core arguments
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="point_lio",
        description="Backend: fast_lio | faster_lio | point_lio",
    )
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    # Backend parameter files (defaults)
    fast_lio_params_arg = DeclareLaunchArgument(
        "fast_lio_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_bringup"),
                "config",
                "fast_lio_mid360.yaml",
            ]
        ),
        description="YAML for fast_lio node",
    )
    faster_lio_params_arg = DeclareLaunchArgument(
        "faster_lio_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_bringup"),
                "config",
                "faster_lio_ros2.yaml",
            ]
        ),
        description="YAML for faster_lio_ros2 node",
    )
    point_lio_ros2_params_arg = DeclareLaunchArgument(
        "point_lio_ros2_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_bringup"),
                "config",
                "point_lio_mid360.yaml",
            ]
        ),
        description="YAML for point_lio_ros2 node",
    )
    # Configurations
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    fast_lio_params = LaunchConfiguration("fast_lio_params")
    faster_lio_params = LaunchConfiguration("faster_lio_params")
    point_lio_ros2_params = LaunchConfiguration("point_lio_ros2_params")

    # Helper for equality conditions
    def equals(var, value):
        return PythonExpression(["'", var, "' == '", value, "'"])

    # Backend odometry nodes (no relocalization)
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fast_lio_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(equals(backend, "fast_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )
    faster_lio_node = Node(
        package="faster_lio_ros2",
        executable="run_mapping_online",
        name="laser_mapping",
        output="screen",
        parameters=[faster_lio_params, {"use_sim_time": use_sim_time}],
        remappings=[("/Odometry", "/odom")],
        condition=IfCondition(equals(backend, "faster_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )
    point_lio_ros2_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("aft_mapped_to_init", "/odom"),
        ],
        parameters=[point_lio_ros2_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(equals(backend, "point_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )
    # For odom-only mode, connect map3d to odom so RViz can display everything.
    # TF: map3d -> odom (for FAST-LIO/FASTER-LIO only)
    tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3d_to_odom",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "map3d", "--child-frame-id", "odom"],
        condition=IfCondition(PythonExpression(["'", backend, "' != 'point_lio'"])),
    )
    
    tf_map3dto2d = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3dto2d",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.25",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "map3d",
        ],
    )

    # TF: map3d -> odom (for Point-LIO only). Point-LIO publishes
    # camera_init -> body, and odom -> camera_init below aliases camera_init to
    # the odom frame, keeping a single TF chain.
    tf_map3d_to_odom_point_lio = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3d_to_odom_point_lio",
        arguments=["0", "0", "0", "0", "0", "0", "map3d", "odom"],
        condition=IfCondition(equals(backend, "point_lio")),
    )

    # For point_lio: camera_init is essentially odom, create alias
    tf_odom_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_odom_to_camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        condition=IfCondition(equals(backend, "point_lio")),
    )

    tf_body2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body2base_link",
        arguments=[
            "--x", "0.0",
            "--y", "0.12848040398218347",
            "--z", "-0.2932452655927712",
            "--roll", "0",
            "--pitch", "0.2617993877991494",
            "--yaw", "1.5707963267948966",
            "--frame-id", "body",
            "--child-frame-id", "base_link",
        ],
    )

    # RViz (optional)
    rviz_config_path = os.path.join(
        get_package_share_directory("rm_bringup"), "rviz", "localize.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription(
        [
            # Args
            backend_arg,
            rviz_arg,
            use_sim_time_arg,
            fast_lio_params_arg,
            faster_lio_params_arg,
            point_lio_ros2_params_arg,
            # Nodes
            fast_lio_node,
            faster_lio_node,
            point_lio_ros2_node,
            tf_map_to_odom,
            tf_map3dto2d,
            tf_map3d_to_odom_point_lio,
            tf_odom_to_camera_init,
            tf_body2base,
            rviz_node,
        ]
    )
