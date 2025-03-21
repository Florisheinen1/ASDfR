cmake_minimum_required(VERSION 3.8)
project(ros_time)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(seq15 src/seq15.cpp)
ament_target_dependencies(seq15 rclcpp std_msgs)

add_executable(loop15 src/loop15.cpp)
ament_target_dependencies(loop15 rclcpp std_msgs)

# Install executables
install(TARGETS
  seq15
  loop15
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for copyright/adding copyright and license to files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
