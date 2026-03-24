#!/usr/bin/env python3
# coding: utf-8

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
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


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="point_lio",
        description="Backend: fast_lio | faster_lio | point_lio",
    )
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    # PCD保存参数
    pcd_save_en_arg = DeclareLaunchArgument("pcd_save_en", default_value="True")
    pcd_save_interval_arg = DeclareLaunchArgument("pcd_save_interval", default_value="-1")
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")[2:]
    default_src_pcd_path = (
        f"/home/xjturm/xjtu_nav25_new/src/rm_bringup/tmp/pcd_{ts}.pcd"
    )
    pcd_save_file_arg = DeclareLaunchArgument(
        "pcd_save_file",
        default_value=default_src_pcd_path,
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

    # 启动全局定位
    run_global_arg = DeclareLaunchArgument(
        "run_global_localization", default_value="true"
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rm_bringup"), "PCD", "RMUL", "RMUL.pcd"]
        ),
    )

    # 全局定位参数
    freq_localization_arg = DeclareLaunchArgument("freq_localization", default_value="2.0") # 重定位频率
    localization_th_arg = DeclareLaunchArgument("localization_th", default_value="0.1")     # MSE匹配阈值
    map_voxel_size_arg = DeclareLaunchArgument("map_voxel_size", default_value="0.05")      # 地图降采样体素大小
    scan_voxel_size_arg = DeclareLaunchArgument("scan_voxel_size", default_value="0.05")    # 扫描降采样体素大小
    fov_arg = DeclareLaunchArgument("fov", default_value="6.28")                            # 视场角（保持360度）
    fov_far_arg = DeclareLaunchArgument("fov_far", default_value="30.0")                    # 远距离视场范围
    use_gicp_arg = DeclareLaunchArgument("use_gicp", default_value="true")                  # 是否使用GICP算法
    
    # 初始位姿参数
    initial_x_arg = DeclareLaunchArgument("initial_x", default_value="0.0")                 # 初始X位置
    initial_y_arg = DeclareLaunchArgument("initial_y", default_value="0.0")                 # 初始Y位置
    initial_z_arg = DeclareLaunchArgument("initial_z", default_value="0.0")                 # 初始Z位置
    initial_yaw_arg = DeclareLaunchArgument("initial_yaw", default_value="0.0")             # 初始航向角(弧度)
    use_initial_pose_arg = DeclareLaunchArgument("use_initial_pose", default_value="false") # 是否使用初始位姿
    enable_multi_hypothesis_arg = DeclareLaunchArgument("enable_multi_hypothesis", default_value="true")  # 多假设初始化
    
    # 全局搜索参数
    enable_global_search_arg = DeclareLaunchArgument("enable_global_search", default_value="true")  # 全局网格搜索
    global_search_step_arg = DeclareLaunchArgument("global_search_step", default_value="2.0")       # 搜索步长(米)
    global_search_yaw_steps_arg = DeclareLaunchArgument("global_search_yaw_steps", default_value="8")  # 航向角搜索数量

    # 启动配置
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pcd_save_en = LaunchConfiguration("pcd_save_en")
    pcd_save_interval = LaunchConfiguration("pcd_save_interval")
    pcd_save_file = LaunchConfiguration("pcd_save_file")
    run_global = LaunchConfiguration("run_global_localization")

    fast_lio_params = LaunchConfiguration("fast_lio_params")
    faster_lio_params = LaunchConfiguration("faster_lio_params")
    point_lio_ros2_params = LaunchConfiguration("point_lio_ros2_params")

    # Common parameter bundle used by all backends
    common_params = {
        "use_sim_time": use_sim_time,
        "pcd_save": {
            "pcd_save_en": pcd_save_en,
            "interval": pcd_save_interval,
            "file_path": pcd_save_file,
        },
    }

    # Backend nodes
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fast_lio_params, common_params],
        remappings=[("/Odometry", "/odom")],
        condition=IfCondition(PythonExpression(["'", backend, "' == 'fast_lio'"])),
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
        condition=IfCondition(PythonExpression(["'", backend, "' == 'faster_lio'"])),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )

    point_lio_ros2_node = Node(
        package="point_lio_ros2",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        parameters=[point_lio_ros2_params, common_params],
        remappings=[
            ("/tf", "tf"), 
            ("/tf_static", "tf_static"), 
            ("/Odometry", "/odom"),
            ("cloud_registered_body", "/cloud_registered_body"),
            ("cloud_registered", "/cloud_registered"),
            ("cloud_effected", "/cloud_effected")
        ],
        condition=IfCondition(PythonExpression(["'", backend, "' == 'point_lio'"])),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )

    # 修复 Point-LIO 发布 camera_init 而不是 odom 的问题
    # 发布静态 TF: odom -> camera_init (重合)
    point_lio_odom_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        condition=IfCondition(PythonExpression(["'", backend, "' == 'point_lio'"])),
    )

    # Map publisher (PCD)
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
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(run_global),
    )

    # Global localization
    global_loc = Node(
        package="fast_lio_localization_ros2",
        executable="global_localization",
        name="global_localization",
        output="screen",
        parameters=[
            {
                "map2odom_completed": False,
                "region": 0,
                "use_sim_time": use_sim_time,
                "map_frame": "map3d",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "freq_localization": LaunchConfiguration("freq_localization"),
                "localization_th": LaunchConfiguration("localization_th"),
                "map_voxel_size": LaunchConfiguration("map_voxel_size"),
                "scan_voxel_size": LaunchConfiguration("scan_voxel_size"),
                "fov": LaunchConfiguration("fov"),
                "fov_far": LaunchConfiguration("fov_far"),
                "use_gicp": LaunchConfiguration("use_gicp"),
                # Initial pose parameters
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_z": LaunchConfiguration("initial_z"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "use_initial_pose": LaunchConfiguration("use_initial_pose"),
                "enable_multi_hypothesis": LaunchConfiguration("enable_multi_hypothesis"),
                # Global grid search parameters
                "enable_global_search": LaunchConfiguration("enable_global_search"),
                "global_search_step": LaunchConfiguration("global_search_step"),
                "global_search_yaw_steps": LaunchConfiguration("global_search_yaw_steps"),
            }
        ],
        condition=IfCondition(run_global),
    )

    # Transform fusion
    transform_fusion = Node(
        package="fast_lio_localization_ros2",
        executable="transform_fusion",
        name="transform_fusion",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map3d",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
            }
        ],
        condition=IfCondition(run_global),
    )

    # Static TFs
    tf_body2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body2base_link",
        arguments=[
            "0.0",
            "0.12848040398218347",
            "-0.2932452655927712",
            "1.5707963267948966",
            "0.2617993877991494",
            "0",
            "body",
            "base_link",
        ],
        condition=IfCondition(run_global),
    )

    tf_map3dto2d = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3dto2d",
        arguments=["0", "0", "0.25", "0", "0", "0", "map", "map3d"],
        condition=IfCondition(run_global),
    )

    tf_base_link2realsense = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link2realsense",
        arguments=["0.3", "0", "0.28", "0", "0", "0", "base_link", "camera_link"],
        condition=IfCondition(run_global),
    )

    # Static TF for point_lio compatibility: odom -> camera_init (identity)
    tf_odom2camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_odom2camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'point_lio'"])
        ),
    )

    # Static TF for point_lio compatibility: aft_mapped -> body (identity)
    tf_aft_mapped2body = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_aft_mapped2body",
        arguments=["0", "0", "0", "0", "0", "0", "aft_mapped", "body"],
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'point_lio'"])
        ),
    )

    # RViz
    rviz_config_path = os.path.join(
        get_package_share_directory("rm_bringup"), "rviz", "localize.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            backend_arg,
            rviz_arg,
            use_sim_time_arg,
            fast_lio_params_arg,
            faster_lio_params_arg,
            point_lio_ros2_params_arg,
            run_global_arg,
            map_arg,
            freq_localization_arg,
            localization_th_arg,
            map_voxel_size_arg,
            scan_voxel_size_arg,
            fov_arg,
            fov_far_arg,
            use_gicp_arg,
            pcd_save_en_arg,
            pcd_save_interval_arg,
            pcd_save_file_arg,
            # Initial pose arguments
            initial_x_arg,
            initial_y_arg,
            initial_z_arg,
            initial_yaw_arg,
            use_initial_pose_arg,
            enable_multi_hypothesis_arg,
            # Global search arguments
            enable_global_search_arg,
            global_search_step_arg,
            global_search_yaw_steps_arg,
            # Nodes
            fast_lio_node,
            faster_lio_node,
            point_lio_ros2_node,
            point_lio_odom_fix,
            pcd_pub,
            global_loc,
            transform_fusion,
            tf_body2base,
            tf_map3dto2d,
            tf_base_link2realsense,
            tf_odom2camera_init,
            tf_aft_mapped2body,
            GroupAction(
                [rviz_node], condition=IfCondition(LaunchConfiguration("rviz"))
            ),
        ]
    )
