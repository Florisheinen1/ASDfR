from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	return LaunchDescription([
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
		# Subs /trackpos and pubs left and right wheel speed
		Node(
			package="bot_controller",
			executable="bot_controller",
			name="bot_controller",
			output="screen"
		)
])