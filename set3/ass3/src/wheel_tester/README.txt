Package wheel_tester
-----------------------------------------------
Description: This package sends target wheel speeds
to the robot via the ros2 xenomai bridge. The goal is
to drive the robot in a straight line, and then rotate
it 90 degrees. This action happens everytime it receives
an empty input.

----------------------------------------------
1. Node wheel_tester

Inputs:
- /XenoState
	Type: std_msgs::msg::Int32
	The current state of the statemachine
- /line_and_turn
	Type: std_msgs::msg::Empty
	Initiates the line and turn action

Outputs:
- /Ros2Xeno
	Type: xrf2_msgs::msg::Ros2Xeno
	The two wheel speeds the robot should have
- /Command
	Type: std_msgs::msg::Int32
	The wanted state of the statemachine

Run wheel_tester:
	ros2 run wheel_tester wheel_tester
