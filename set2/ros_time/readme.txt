Paste the package in your ROS 2 workspace
Build the package using colcon build
Ensure to source ROS and install/setup.bash

To run the node seq15 use: ros2 run ros_time seq15 
To run the node loop15 use: ros2 run ros_time loop15

To run with the required QoS parameters use the following after the usual way of running nodes (refer to useage below): --ros-args 

example: ros2 run ros_time seq15   --ros-args -p reliability:=best_effort -p history:=keep_last -p depth:=50 -p durability:=volatile

To work with the program, run both loop15 and seq15. A timing_results.csv file is output with the observations of jitter and rtt.
Note: The nodes have to be manually stopped.