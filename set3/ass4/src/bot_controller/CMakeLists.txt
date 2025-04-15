cmake_minimum_required(VERSION 3.8)
project(bot_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xrf2_msgs REQUIRED)


include_directories(include)

add_executable(bot_controller src/bot_controller.cpp)


ament_target_dependencies(bot_controller 
  rclcpp
  geometry_msgs
  std_msgs
  xrf2_msgs
)

# Install executables
install(TARGETS 
  bot_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()