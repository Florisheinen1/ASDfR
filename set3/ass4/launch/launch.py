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
		# Run the wheel tester
		Node(
			package='wheel_tester',
			executable='wheel_tester',
			name='wheel_tester',
			output="screen",
		),
		# Start initializing in the statemachine
		ExecuteProcess(
			cmd=[
				"ros2", "topic", "pub", "--once", "/Command", "std_msgs/msg/Int32", "{data: 1}"
			],
			output="screen",
		),
	])