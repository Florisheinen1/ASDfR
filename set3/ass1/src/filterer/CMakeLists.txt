cmake_minimum_required(VERSION 3.8)
project(filterer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(greyscaler src/greyscaler.cpp)
add_executable(masker src/masker.cpp)
ament_target_dependencies(greyscaler rclcpp std_msgs sensor_msgs)
ament_target_dependencies(masker rclcpp std_msgs sensor_msgs)

install(TARGETS
  greyscaler
  masker
  DESTINATION lib/${PROJECT_NAME})

ament_package()