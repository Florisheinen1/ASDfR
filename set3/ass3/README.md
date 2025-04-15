Assignment 3 workspace

Description:
Contains our ros test node and the ros-xenomai framework,
with our implementation of the states of the statemachine.
Purpose: Make the robot drive in a straight line, and then
turn 90 degrees.

-----------------------------------------------
1. Run

	Start the group15 statemachine implementation:
	`sudo ./build/group15/group15`

	Launch the launch file:
	`ros2 launch launch/launch.py`

	Initiate the driving:
	`ros2 topic pub --once /line_and_turn std_msgs/msg/Empty "{}"`