
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the RELbot Simulator with required parameters
        Node(
            package="relbot_simulator",
            executable="relbot_simulator",
            name="relbot_simulator",
            output="screen",
            parameters=[
                {"code": 30},  # Set image processing FPS
                {"use_twist_cmd": True}  # Enable Twist control
            ]
        ),

        # Start the sequence_controller node
        Node(
            package="bot_controller",
            executable="sequence_controller",
            name="sequence_controller",
            output="screen",
            parameters=[
                {"use_twist_cmd": True}  # Ensure consistency with the simulator
            ]
        )
    ])
