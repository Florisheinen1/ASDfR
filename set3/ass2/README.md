Assignment 2 workspace

Description:
Contains our ros test node and the ros-xenomai framework,
with our implementation of the states of the statemachine.

-----------------------------------------------
1. Run

	Start the group15 statemachine implementation:
	`sudo ./build/group15/group15`

	Start the ros-xenomai bridge
	`ros2 run ros_xeno_bridge RosXenoBridge`

	Start the webcam image publisher:
	`ros2 run cam2image_vm2ros cam2image`

	Inside the assignment workspace folder:
	`ros2 launch launch/launch.py`
