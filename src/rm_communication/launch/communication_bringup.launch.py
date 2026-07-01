from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    reopen_interval_ms = LaunchConfiguration("reopen_interval_ms")
    read_loop_hz = LaunchConfiguration("read_loop_hz")
    tx_hz = LaunchConfiguration("tx_hz")
    cmd_vel_frame = LaunchConfiguration("cmd_vel_frame")
    angular_z_mode = LaunchConfiguration("angular_z_mode")
    yaw_rate_preview_time = LaunchConfiguration("yaw_rate_preview_time")
    cmd_vel_predict_time = LaunchConfiguration("cmd_vel_predict_time")
    wz_filter_tau = LaunchConfiguration("wz_filter_tau")
    smooth_world_velocity = LaunchConfiguration("smooth_world_velocity")
    world_velocity_filter_tau = LaunchConfiguration("world_velocity_filter_tau")
    world_velocity_accel_limit = LaunchConfiguration("world_velocity_accel_limit")
    sim_serial = LaunchConfiguration("sim_serial")
    sim_publish_hz = LaunchConfiguration("sim_publish_hz")
    sim_state = LaunchConfiguration("sim_state")
    sim_patrol_region = LaunchConfiguration("sim_patrol_region")
    sim_auto_sequence = LaunchConfiguration("sim_auto_sequence")
    sim_sequence_period_s = LaunchConfiguration("sim_sequence_period_s")
    sim_control_panel = LaunchConfiguration("sim_control_panel")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument(
                "reopen_interval_ms", default_value="500"
            ),
            DeclareLaunchArgument("read_loop_hz", default_value="200.0"),
            DeclareLaunchArgument("tx_hz", default_value="100.0"),
            DeclareLaunchArgument("cmd_vel_frame", default_value="map"),
            DeclareLaunchArgument("angular_z_mode", default_value="yaw_angle"),
            DeclareLaunchArgument("yaw_rate_preview_time", default_value="0.15"),
            DeclareLaunchArgument("cmd_vel_predict_time", default_value="0.08"),
            DeclareLaunchArgument("wz_filter_tau", default_value="0.05"),
            DeclareLaunchArgument("smooth_world_velocity", default_value="true"),
            DeclareLaunchArgument("world_velocity_filter_tau", default_value="0.12"),
            DeclareLaunchArgument("world_velocity_accel_limit", default_value="1.2"),
            DeclareLaunchArgument("sim_serial", default_value="false"),
            DeclareLaunchArgument("sim_publish_hz", default_value="20.0"),
            DeclareLaunchArgument("sim_state", default_value="0"),
            DeclareLaunchArgument("sim_patrol_region", default_value="1"),
            DeclareLaunchArgument("sim_auto_sequence", default_value="false"),
            DeclareLaunchArgument("sim_sequence_period_s", default_value="5.0"),
            DeclareLaunchArgument("sim_control_panel", default_value="false"),
            Node(
                package="rm_communication",
                executable="serial_rw_node",
                name="serial_rw_node",
                condition=UnlessCondition(sim_serial),
                parameters=[
                    {
                        "port": port,
                        "baud": baud,
                        "reopen_interval_ms": reopen_interval_ms,
                        "read_loop_hz": read_loop_hz,
                    }
                ],
            ),
            Node(
                package="rm_communication",
                executable="virtual_serial_node",
                name="virtual_serial_node",
                condition=IfCondition(sim_serial),
                parameters=[
                    {
                        "publish_hz": sim_publish_hz,
                        "state": sim_state,
                        "patrol_region": sim_patrol_region,
                        "auto_sequence": sim_auto_sequence,
                        "sequence_period_s": sim_sequence_period_s,
                    }
                ],
            ),
            Node(
                package="rm_communication",
                executable="handler_node",
                name="handler_node",
                parameters=[{
                    "tx_hz": tx_hz,
                    "cmd_vel_frame": cmd_vel_frame,
                    "angular_z_mode": angular_z_mode,
                    "yaw_rate_preview_time": yaw_rate_preview_time,
                    "cmd_vel_predict_time": cmd_vel_predict_time,
                    "wz_filter_tau": wz_filter_tau,
                    "smooth_world_velocity": smooth_world_velocity,
                    "world_velocity_filter_tau": world_velocity_filter_tau,
                    "world_velocity_accel_limit": world_velocity_accel_limit,
                }],
            ),
            Node(
                package="rm_communication",
                executable="sim_control_panel.py",
                name="sim_control_panel",
                condition=IfCondition(sim_control_panel),
                output="screen",
            ),
        ]
    )
