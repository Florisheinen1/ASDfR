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

# Include directories
include_directories(include)

# Add executables
add_executable(sequence_controller src/sequence_controller.cpp)
add_executable(imgsqcontrol src/imgsqcontrol.cpp)
add_executable(bot_controller src/bot_controller.cpp)

# Link dependencies
ament_target_dependencies(sequence_controller 
  rclcpp 
  geometry_msgs 
  example_interfaces
)

ament_target_dependencies(imgsqcontrol 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  OpenCV 
  cv_bridge
)

ament_target_dependencies(bot_controller 
  rclcpp 
  geometry_msgs
  example_interfaces
)

# Install executables
install(TARGETS 
  sequence_controller
  imgsqcontrol
  bot_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
