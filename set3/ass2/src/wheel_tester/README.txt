Package wheel_tester
-----------------------------------------------
Description: This package sends target wheel speeds
to the robot via the ros2 xenomai bridge.

----------------------------------------------
1. Node wheel_tester
Every once in a while, swap the wheel speed of the left and
right wheels between stop and moving.

Outputs:
- /Ros2Xeno
	Type: xrf2_msgs::msg::Ros2Xeno
	The two wheel speeds the robot should have

Run wheel_tester:
	ros2 run wheel_tester wheel_tester
