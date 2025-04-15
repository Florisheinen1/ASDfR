from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	return LaunchDescription([
		# Run the ros-xenomai bridge
		Node(
			package='ros_xeno_bridge',
			executable='RosXenoBridge',
			name='ros_xeno_bridge',
			output="screen",
		),
		# Start initializing in the statemachine
		ExecuteProcess(
			cmd=[
				"ros2", "topic", "pub", "--once", "/Command", "std_msgs/msg/Int32", "{data: 1}"
			],
			output="screen",
		),
		# Run the bot controller
		Node(
			package='bot_controller',
			executable='bot_controller',
			name='bot_controller',
			output="screen",
		),
		# Subs /image and pubs green-masked image /mask
		Node(
			package='filterer',
			executable='masker',
			name='masker',
			output='screen'
		),
		# Subs /mask and pubs center of gravity on /trackpos
		Node(
			package='tracker',
			executable='tracker',
			name='tracker',
			output='screen'
		),
	])