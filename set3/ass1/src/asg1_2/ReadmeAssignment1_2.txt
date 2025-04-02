# Assignment 1.2

1. Source the ros2 environment in all new terminals
2. paste the packages asg1_2 and relbot simulator along with packages of section 1.
3. Navigate to the workspace directory and build the project using: colcon build.
4. source install/setup.bash for new packages to be available.

Note: Open new terminal and source the ros2 environment for each new point below:
Section 1.2.1
1. launch node relbot_simulator with the arguments:  ros2 run relbot_simulator relbot_simulator --ros-args -p use_twist_cmd:=true
2. launch the manual setpoint controller : ros2 run asg1_2 sequence_controller
3. to see the graph use rqt

Section 1.2.2
1. launch node relbot_simulator with the arguments:  ros2 run relbot_simulator relbot_simulator --ros-args -p use_twist_cmd:=true
2. launch the manual setpoint controller : ros2 run asg1_2 sequence_controller
3. to see the graph use rqt

Section 1.2.2
1. launch the cam2image node to receive data from the camera: ros2 run cam2image_vm2ros cam2image
2. launch showimage to see what we are working with: ros2 run image_tools showimage
3. launch the grayscale filter to convert the image: ros2 run filterer greyscaler
4. launch node relbot_simulator with the arguments:  ros2 run relbot_simulator relbot_simulator --ros-args -p use_twist_cmd:=true
5. launch the image sequence controller: ros2 run asg1_2 imgsqcontrol
6. Use rqt to visualize 


