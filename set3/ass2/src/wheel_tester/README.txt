Package wheel_tester
-----------------------------------------------
Description: This package sends target wheel speeds
to the robot via the ros2 xenomai bridge.

----------------------------------------------
1. Node wheel_tester
Every once in a while, swap the wheel speed of the left and
right wheels between stop and moving. Additionally, publishes the
RUN command to the statemachine when it is done initializing.

Inputs:
- /XenoState
	Type: std_msgs::msg::Int32
	The current state of the statemachine

Outputs:
- /Ros2Xeno
	Type: xrf2_msgs::msg::Ros2Xeno
	The two wheel speeds the robot should have
- /Command
	Type: std_msgs::msg::Int32
	The wanted state of the statemachine

Run wheel_tester:
	ros2 run wheel_tester wheel_tester
