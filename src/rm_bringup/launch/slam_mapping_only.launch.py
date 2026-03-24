#!/usr/bin/env python3
# coding: utf-8

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from datetime import datetime
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="point_lio",
        description="Backend: fast_lio | faster_lio | point_lio",
    )
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    # 这里没有rosbag播放；如果需要，请外部运行rosbag
    record_rosbag_arg = DeclareLaunchArgument(
        "record_rosbag",
        default_value="false",
        description="Record /livox/lidar and /livox/imu",
    )
    record_output_arg = DeclareLaunchArgument(
        "record_output",
        default_value="rosbags/mapping_record",
        description="Output prefix/path for rosbag record",
    )

    # PCD保存参数
    pcd_save_en_arg = DeclareLaunchArgument("pcd_save_en", default_value="True")
    pcd_save_interval_arg = DeclareLaunchArgument("pcd_save_interval",default_value="-1")
    LAUNCH_FILE = Path(__file__).resolve()  # 若为 symlink-install，则解析到 install 目录
    LAUNCH_DIR = LAUNCH_FILE.parent  # your_pkg/launch
    PKG_ROOT = LAUNCH_DIR.parent  # your_pkg （源码根，若 symlink-install）
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    tmp_path = PKG_ROOT / "tmp"
    tmp_path.mkdir(parents=True, exist_ok=True)
    default_pcd_path = tmp_path / f"scans_{ts}.pcd"
    pcd_save_file_arg = DeclareLaunchArgument(
        "pcd_save_file",
        default_value=str(default_pcd_path),
    )

    # 各个后端的参数文件，都默认放在 rm_bringup/config 目录下
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

    # 启动配置
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    record_rosbag = LaunchConfiguration("record_rosbag")
    record_output = LaunchConfiguration("record_output")
    pcd_save_en = LaunchConfiguration("pcd_save_en")
    pcd_save_interval = LaunchConfiguration("pcd_save_interval")
    pcd_save_file = LaunchConfiguration("pcd_save_file")

    fast_lio_params = LaunchConfiguration("fast_lio_params")
    faster_lio_params = LaunchConfiguration("faster_lio_params")
    point_lio_ros2_params = LaunchConfiguration("point_lio_ros2_params")

    # Small helper to build equality condition "var == value"
    def equals(var, value):
        return PythonExpression(["'", var, "' == '", value, "'"])

    # Common parameter bundle used by all backends
    common_params = {
        "use_sim_time": use_sim_time,
        "pcd_save": {
            "pcd_save_en": pcd_save_en,
            "interval": pcd_save_interval,
            "file_path": pcd_save_file,
        },
    }

    # Backend nodes (mapping only)
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fast_lio_params, common_params],
        remappings=[("/Odometry", "/odom")],
        condition=IfCondition(equals(backend, "fast_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )

    faster_lio_node = Node(
        package="faster_lio_ros2",
        executable="run_mapping_online",
        name="laser_mapping",
        output="screen",
        parameters=[faster_lio_params, common_params],
        remappings=[("/Odometry", "/odom")],
        condition=IfCondition(equals(backend, "faster_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )

    point_lio_ros2_node = Node(
        package="point_lio_ros2",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        parameters=[point_lio_ros2_params, common_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("/Odometry", "/odom")],
        condition=IfCondition(equals(backend, "point_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )

    # Static TF for point_lio compatibility: odom -> camera_init (identity)
    tf_odom2camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_odom2camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        condition=IfCondition(equals(backend, "point_lio")),
    )

    # Optional recording of Live/Bag topics
    bag_record_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            record_output,
            "/livox/lidar",
            "/livox/imu",
        ],
        output="screen",
        condition=IfCondition(record_rosbag),
    )

    # RViz (optional)
    rviz_config_path = os.path.join(
        get_package_share_directory("rm_bringup"), "rviz", "mapping.rviz"
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
            record_rosbag_arg,
            record_output_arg,
            pcd_save_en_arg,
            pcd_save_interval_arg,
            pcd_save_file_arg,
            fast_lio_params_arg,
            faster_lio_params_arg,
            point_lio_ros2_params_arg,
            # Mapping backends
            fast_lio_node,
            faster_lio_node,
            point_lio_ros2_node,
            tf_odom2camera_init,
            # Data source/recording
            bag_record_process,
            # RViz
            rviz_node,
        ]
    )
