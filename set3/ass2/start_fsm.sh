ros2 topic pub --once /Command std_msgs/msg/Int32 "{data: 1}"

sleep 4

ros2 topic pub --once /Command std_msgs/msg/Int32 "{data: 2}"