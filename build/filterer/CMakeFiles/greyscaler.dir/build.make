# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/man1/ASDfR/set1/src/filterer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/man1/ASDfR/build/filterer

# Include any dependencies generated for this target.
include CMakeFiles/greyscaler.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/greyscaler.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/greyscaler.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/greyscaler.dir/flags.make

CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o: CMakeFiles/greyscaler.dir/flags.make
CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o: /home/man1/ASDfR/set1/src/filterer/src/greyscaler.cpp
CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o: CMakeFiles/greyscaler.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/man1/ASDfR/build/filterer/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o -MF CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o.d -o CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o -c /home/man1/ASDfR/set1/src/filterer/src/greyscaler.cpp

CMakeFiles/greyscaler.dir/src/greyscaler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/greyscaler.dir/src/greyscaler.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/man1/ASDfR/set1/src/filterer/src/greyscaler.cpp > CMakeFiles/greyscaler.dir/src/greyscaler.cpp.i

CMakeFiles/greyscaler.dir/src/greyscaler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/greyscaler.dir/src/greyscaler.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/man1/ASDfR/set1/src/filterer/src/greyscaler.cpp -o CMakeFiles/greyscaler.dir/src/greyscaler.cpp.s

# Object files for target greyscaler
greyscaler_OBJECTS = \
"CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o"

# External object files for target greyscaler
greyscaler_EXTERNAL_OBJECTS =

greyscaler: CMakeFiles/greyscaler.dir/src/greyscaler.cpp.o
greyscaler: CMakeFiles/greyscaler.dir/build.make
greyscaler: /opt/ros/jazzy/lib/librclcpp.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_generator_py.so
greyscaler: /opt/ros/jazzy/lib/liblibstatistics_collector.so
greyscaler: /opt/ros/jazzy/lib/librcl.so
greyscaler: /opt/ros/jazzy/lib/librmw_implementation.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_generator_py.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_generator_py.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/librcl_yaml_param_parser.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_generator_py.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_generator_py.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/libtracetools.so
greyscaler: /opt/ros/jazzy/lib/librcl_logging_interface.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_generator_py.so
greyscaler: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/librosidl_typesupport_fastrtps_c.so
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/librosidl_typesupport_fastrtps_cpp.so
greyscaler: /opt/ros/jazzy/lib/librmw.so
greyscaler: /opt/ros/jazzy/lib/librosidl_dynamic_typesupport.so
greyscaler: /opt/ros/jazzy/lib/libfastcdr.so.2.2.5
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/librosidl_typesupport_introspection_cpp.so
greyscaler: /opt/ros/jazzy/lib/librosidl_typesupport_introspection_c.so
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_generator_py.so
greyscaler: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/libstd_msgs__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/librosidl_typesupport_cpp.so
greyscaler: /opt/ros/jazzy/lib/libservice_msgs__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_generator_c.so
greyscaler: /opt/ros/jazzy/lib/librosidl_typesupport_c.so
greyscaler: /opt/ros/jazzy/lib/librcpputils.so
greyscaler: /opt/ros/jazzy/lib/librosidl_runtime_c.so
greyscaler: /opt/ros/jazzy/lib/librcutils.so
greyscaler: CMakeFiles/greyscaler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/man1/ASDfR/build/filterer/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable greyscaler"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/greyscaler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/greyscaler.dir/build: greyscaler
.PHONY : CMakeFiles/greyscaler.dir/build

CMakeFiles/greyscaler.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/greyscaler.dir/cmake_clean.cmake
.PHONY : CMakeFiles/greyscaler.dir/clean

CMakeFiles/greyscaler.dir/depend:
	cd /home/man1/ASDfR/build/filterer && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/man1/ASDfR/set1/src/filterer /home/man1/ASDfR/set1/src/filterer /home/man1/ASDfR/build/filterer /home/man1/ASDfR/build/filterer /home/man1/ASDfR/build/filterer/CMakeFiles/greyscaler.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/greyscaler.dir/depend

