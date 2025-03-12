cmake_minimum_required(VERSION 3.8)
project(asg1_2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Add include directories
include_directories(include)

# Add executables
add_executable(sequence_controller src/sequence_controller.cpp)
add_executable(imgsqcontrol src/imgsqcontrol.cpp)
add_executable(relbot_controller src/1_2_3.cpp)  # Added new executable

# Link dependencies for sequence_controller
ament_target_dependencies(sequence_controller rclcpp geometry_msgs example_interfaces)

# Link dependencies for imgsqcontrol
ament_target_dependencies(imgsqcontrol 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  OpenCV 
  cv_bridge
)

# Link dependencies for relbot_controller
ament_target_dependencies(relbot_controller 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  OpenCV 
  cv_bridge
)

# Install the executables
install(TARGETS 
  sequence_controller
  imgsqcontrol
  relbot_controller  # Added new executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
