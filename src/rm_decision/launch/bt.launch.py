from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
	pkg_share = get_package_share_directory('rm_decision')
	default_tree = os.path.join(pkg_share, 'config', 'trees', 'siyuan.yaml')

	tree_arg = DeclareLaunchArgument(
		'tree', default_value=default_tree,
		description='Path to BT YAML file'
	)
	tick_hz_arg = DeclareLaunchArgument('tick_hz', default_value='5.0')
	use_web_arg = DeclareLaunchArgument('use_web_viewer', default_value='true')
	text_arg = DeclareLaunchArgument('enable_text_output', default_value='true')
	text_every_arg = DeclareLaunchArgument('text_output_every_n', default_value='1')

	node = Node(
		package='rm_decision',
		executable='bt_node',
		name='rm_bt_decision_node',
		parameters=[{
			'tree_config': LaunchConfiguration('tree'),
			'tick_hz': LaunchConfiguration('tick_hz'),
			'use_web_viewer': LaunchConfiguration('use_web_viewer'),
			'enable_text_output': LaunchConfiguration('enable_text_output'),
			'text_output_every_n': LaunchConfiguration('text_output_every_n'),
		}],
		output='screen'
	)

	return LaunchDescription([
		tree_arg,
		tick_hz_arg,
		use_web_arg,
		text_arg,
		text_every_arg,
		node,
	]) 