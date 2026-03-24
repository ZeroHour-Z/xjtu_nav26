from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import LogInfo


def generate_launch_description():
    use_sim_time = False
    autostart = True

    # 默认地图路径
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("rm_bringup"), "PCD", "RMUL", "RMUL.yaml"],
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='要加载的地图 yaml 文件的完整路径'
    )

    map_yaml_param = ParameterValue(LaunchConfiguration('map'), value_type=str)


    default_params = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "config", "nav2_params.yaml"]
    )

    default_bt_xml = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ]
    )

    default_nav_to_pose_bt_xml = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "config", "bt.xml"]
    )

    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "rviz", "nav2_view.rviz"]
    )

    gvc_params = PathJoinSubstitution(
        [
            FindPackageShare("global_velocity_controller"),
            "config",
            "controller_params.yaml",
        ]
    )
    gvc_tracker_params = PathJoinSubstitution(
        [
            FindPackageShare("global_velocity_controller"),
            "config",
            "tracker_params.yaml",
        ]
    )
    gvc_sim_params = PathJoinSubstitution(
        [
            FindPackageShare("global_velocity_controller"),
            "config",
            "simulator_params.yaml",
        ]
    )

    nodes = [
        map_arg,
        LogInfo(msg=["Loading map from: ", LaunchConfiguration('map')]),
    ]

    # 直接使用 substitution（让 launch 在运行时解析为绝对路径）
    nodes.append(
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_param}
            ],
        )
    )

    # 静态 TF 变换: map -> odom
    # nodes.append(
    #     Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="static_map_to_odom",
    #         output="screen",
    #         arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    #     )
    # )

    nodes.extend(
        [
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[default_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[default_params, {"use_sim_time": use_sim_time}],
                # Nav2输出到中间话题，GVC处理后输出到/cmd_vel
                remappings=[("/cmd_vel", "/nav2_cmd_vel")],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[default_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    default_params,
                    {
                        "use_sim_time": use_sim_time,
                        "default_bt_xml_filename": default_bt_xml,
                        "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                    },
                ],
            ),
        ]
    )

    node_names = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "map_server",
    ]

    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": node_names,
                }
            ],
        )
    )

    # GVC作为中间层，订阅Nav2的输出并发布到底盘
    nodes.append(
        Node(
            package="global_velocity_controller",
            executable="global_velocity_controller_node",
            name="global_velocity_controller",
            output="screen",
            parameters=[
                gvc_params,
                gvc_tracker_params,
                gvc_sim_params,
                {"use_sim_time": use_sim_time, "simulate": False},
            ],
        )
    )

    # nodes.append(
    #     Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         name="rviz2",
    #         output="screen",
    #         arguments=["-d", default_rviz_config],
    #     )
    # )

    return LaunchDescription(nodes)
