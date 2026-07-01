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
    rx_diagnostics = LaunchConfiguration("rx_diagnostics")
    nav_rx_log_period_ms = LaunchConfiguration("nav_rx_log_period_ms")
    aim_rx_log_period_ms = LaunchConfiguration("aim_rx_log_period_ms")
    tx_hz = LaunchConfiguration("tx_hz")
    tx_log_period_ms = LaunchConfiguration("tx_log_period_ms")
    cmd_vel_frame = LaunchConfiguration("cmd_vel_frame")
    angular_z_mode = LaunchConfiguration("angular_z_mode")
    yaw_rate_preview_time = LaunchConfiguration("yaw_rate_preview_time")
    cmd_vel_predict_time = LaunchConfiguration("cmd_vel_predict_time")
    wz_filter_tau = LaunchConfiguration("wz_filter_tau")
    smooth_world_velocity = LaunchConfiguration("smooth_world_velocity")
    world_velocity_filter_tau = LaunchConfiguration("world_velocity_filter_tau")
    world_velocity_accel_limit = LaunchConfiguration("world_velocity_accel_limit")
    chassis_trapped_radius = LaunchConfiguration("chassis_trapped_radius")
    chassis_trapped_timeout = LaunchConfiguration("chassis_trapped_timeout")
    chassis_trapped_goal_tolerance = LaunchConfiguration("chassis_trapped_goal_tolerance")
    motion_disallowed_clear_timeout = LaunchConfiguration("motion_disallowed_clear_timeout")
    sim_serial = LaunchConfiguration("sim_serial")
    sim_publish_hz = LaunchConfiguration("sim_publish_hz")
    sim_rx_log_period_ms = LaunchConfiguration("sim_rx_log_period_ms")
    sim_tx_log_period_ms = LaunchConfiguration("sim_tx_log_period_ms")
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
            DeclareLaunchArgument("rx_diagnostics", default_value="false"),
            DeclareLaunchArgument("nav_rx_log_period_ms", default_value="1000"),
            DeclareLaunchArgument("aim_rx_log_period_ms", default_value="5000"),
            DeclareLaunchArgument("tx_hz", default_value="100.0"),
            DeclareLaunchArgument("tx_log_period_ms", default_value="1000"),
            DeclareLaunchArgument("cmd_vel_frame", default_value="map"),
            DeclareLaunchArgument("angular_z_mode", default_value="yaw_angle"),
            DeclareLaunchArgument("yaw_rate_preview_time", default_value="0.15"),
            DeclareLaunchArgument("cmd_vel_predict_time", default_value="0.08"),
            DeclareLaunchArgument("wz_filter_tau", default_value="0.05"),
            DeclareLaunchArgument("smooth_world_velocity", default_value="true"),
            DeclareLaunchArgument("world_velocity_filter_tau", default_value="0.12"),
            DeclareLaunchArgument("world_velocity_accel_limit", default_value="1.2"),
            DeclareLaunchArgument("chassis_trapped_radius", default_value="1.0"),
            DeclareLaunchArgument("chassis_trapped_timeout", default_value="30.0"),
            DeclareLaunchArgument("chassis_trapped_goal_tolerance", default_value="0.30"),
            DeclareLaunchArgument("motion_disallowed_clear_timeout", default_value="5.0"),
            DeclareLaunchArgument("sim_serial", default_value="false"),
            DeclareLaunchArgument("sim_publish_hz", default_value="20.0"),
            DeclareLaunchArgument("sim_rx_log_period_ms", default_value="1000"),
            DeclareLaunchArgument("sim_tx_log_period_ms", default_value="1000"),
            DeclareLaunchArgument("sim_state", default_value="0"),
            DeclareLaunchArgument("sim_patrol_region", default_value="1"),
            DeclareLaunchArgument("sim_auto_sequence", default_value="false"),
            DeclareLaunchArgument("sim_sequence_period_s", default_value="5.0"),
            DeclareLaunchArgument("sim_control_panel", default_value="false"),
            Node(
                package="rm_communication",
                executable="serial_rw_node",
                name="serial_rw_node",
                output="screen",
                condition=UnlessCondition(sim_serial),
                parameters=[
                    {
                        "port": port,
                        "baud": baud,
                        "reopen_interval_ms": reopen_interval_ms,
                        "read_loop_hz": read_loop_hz,
                        "rx_diagnostics": rx_diagnostics,
                        "nav_rx_log_period_ms": nav_rx_log_period_ms,
                        "aim_rx_log_period_ms": aim_rx_log_period_ms,
                    }
                ],
            ),
            Node(
                package="rm_communication",
                executable="virtual_serial_node",
                name="virtual_serial_node",
                output="screen",
                condition=IfCondition(sim_serial),
                parameters=[
                    {
                        "publish_hz": sim_publish_hz,
                        "sim_rx_log_period_ms": sim_rx_log_period_ms,
                        "sim_tx_log_period_ms": sim_tx_log_period_ms,
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
                output="screen",
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
                    "tx_log_period_ms": tx_log_period_ms,
                    "chassis_trapped_radius": chassis_trapped_radius,
                    "chassis_trapped_timeout": chassis_trapped_timeout,
                    "chassis_trapped_goal_tolerance": chassis_trapped_goal_tolerance,
                    "motion_disallowed_clear_timeout": motion_disallowed_clear_timeout,
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
