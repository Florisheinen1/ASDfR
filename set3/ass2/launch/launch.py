from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Run a node directly
        Node(
            package='filterer',
            executable='masker',
            name='masker',
            output='screen'
        ),

        Node(
            package='tracker',
            executable='tracker',
            name='tracker',
            output='screen'
        ),

	Node(
		package="asg1_2",
		executable="bot_controller",
		name="bot_controller",
		output="screen"
	)
])