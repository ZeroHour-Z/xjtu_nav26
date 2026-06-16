from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bind_host = LaunchConfiguration("bind_host")
    peer_host = LaunchConfiguration("peer_host")
    aim_to_nav_port = LaunchConfiguration("aim_to_nav_port")
    nav_to_aim_port = LaunchConfiguration("nav_to_aim_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("peer_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("aim_to_nav_port", default_value="47001"),
            DeclareLaunchArgument("nav_to_aim_port", default_value="47002"),
            Node(
                package="rm_communication",
                executable="aim_udp_bridge_node",
                name="aim_udp_bridge_node",
                parameters=[
                    {
                        "bind_host": bind_host,
                        "peer_host": peer_host,
                        "aim_to_nav_port": aim_to_nav_port,
                        "nav_to_aim_port": nav_to_aim_port,
                    }
                ],
            ),
        ]
    )
