#!/usr/bin/env python3
# coding: utf-8

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rm_localization_bringup"), "PCD", "red", "map.pcd"]
        ),
    )

    # Point-LIO node
    point_lio_ros2_dir = get_package_share_directory("point_lio_ros2")
    point_lio_ros2_cfg = os.path.join(point_lio_ros2_dir, "config", "build_map.yaml")
    point_lio_ros2_params = {
        "use_imu_as_input": False,
        "prop_at_freq_of_imu": False,
        "check_satu": True,
        "init_map_size": 10,
        "point_filter_num": 3,
        "space_down_sample": True,
        "filter_size_surf": 0.1,
        "filter_size_map": 0.2,
        "cube_side_length": 500.0,
        "runtime_pos_log_enable": False,
        "use_sim_time": False,
    }
    point_lio_ros2_params_list = []

    # Append YAML if exists
    if os.path.exists(point_lio_ros2_cfg):
        point_lio_ros2_params_list.append(point_lio_ros2_cfg)
    point_lio_ros2_params_list.append(point_lio_ros2_params)

    point_lio_ros2_node = Node(
        package="point_lio_ros2",
        executable="pointlio_mapping",
        parameters=point_lio_ros2_params_list,
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
    )

    # Map publisher (PCD) -> use C++ executable
    pcd_pub = Node(
        package="fast_lio_localization_ros2",
        executable="pcd_publisher",
        name="map_publisher",
        output="screen",
        parameters=[
            {
                "map": LaunchConfiguration("map"),
                "frame_id": "map3d",
                "rate": 1.0,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # Global localization -> use C++ executable
    global_loc = Node(
        package="fast_lio_localization_ros2",
        executable="global_localization",
        name="global_localization",
        output="screen",
        parameters=[
            {
                "map2odom_completed": False,
                "region": 0,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_frame": "map3d",
                "odom_frame": "camera_init",
                "base_link_frame": "base_link",
            }
        ],
    )

    # Transform fusion -> use C++ executable
    transform_fusion = Node(
        package="fast_lio_localization_ros2",
        executable="transform_fusion",
        name="transform_fusion",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_frame": "map3d",
                "odom_frame": "camera_init",
                "base_link_frame": "base_link",
            }
        ],
    )

    # Static TFs (replicating ROS1 args; ROS2 tool doesn't need frequency)
    # body -> base_link
    tf_body2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body2base_link",
        arguments=[
            "0.0",
            "0.2848040398218347",
            "-0.2932452655927712",
            "1.5707963267948966",
            "0.2617993877991494",
            "0",
            "body",
            "base_link",
        ],
    )

    # map (2d) -> map3d
    tf_map3dto2d = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3dto2d",
        arguments=["0", "0", "0.25", "-1.5707963267948966", "0", "0", "map", "map3d"],
    )

    # base_link -> camera_link
    tf_base_link2realsense = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link2realsense",
        arguments=["0.3", "0", "0.28", "0", "0", "0", "base_link", "camera_link"],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("point_lio_ros2"), "rviz_cfg", "localize.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            rviz_arg,
            use_sim_time_arg,
            map_arg,
            pcd_pub,
            global_loc,
            transform_fusion,
            tf_body2base,
            tf_map3dto2d,
            tf_base_link2realsense,
            GroupAction(
                [rviz_node], condition=IfCondition(LaunchConfiguration("rviz"))
            ),
        ]
    )
