# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/workspace/src/drone

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/workspace/build/drone

# Include any dependencies generated for this target.
include CMakeFiles/node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/node.dir/flags.make

CMakeFiles/node.dir/src/offb_node.cpp.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/src/offb_node.cpp.o: /root/workspace/src/drone/src/offb_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/workspace/build/drone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/node.dir/src/offb_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/node.dir/src/offb_node.cpp.o -c /root/workspace/src/drone/src/offb_node.cpp

CMakeFiles/node.dir/src/offb_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/node.dir/src/offb_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/workspace/src/drone/src/offb_node.cpp > CMakeFiles/node.dir/src/offb_node.cpp.i

CMakeFiles/node.dir/src/offb_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/node.dir/src/offb_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/workspace/src/drone/src/offb_node.cpp -o CMakeFiles/node.dir/src/offb_node.cpp.s

# Object files for target node
node_OBJECTS = \
"CMakeFiles/node.dir/src/offb_node.cpp.o"

# External object files for target node
node_EXTERNAL_OBJECTS =

node: CMakeFiles/node.dir/src/offb_node.cpp.o
node: CMakeFiles/node.dir/build.make
node: /opt/ros/foxy/lib/librclcpp.so
node: /root/microros_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_introspection_c.so
node: /root/microros_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_c.so
node: /root/microros_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_introspection_cpp.so
node: /root/microros_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/liblibstatistics_collector.so
node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
node: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/librcl.so
node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/librmw_implementation.so
node: /opt/ros/foxy/lib/librmw.so
node: /opt/ros/foxy/lib/librcl_logging_spdlog.so
node: /root/microros_ws/install/micro_ros_agent/lib/libspdlog.a
node: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
node: /opt/ros/foxy/lib/libyaml.so
node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/libtracetools.so
node: /root/microros_ws/install/px4_msgs/lib/libpx4_msgs__rosidl_generator_c.so
node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
node: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
node: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
node: /opt/ros/foxy/lib/librosidl_typesupport_c.so
node: /opt/ros/foxy/lib/librcpputils.so
node: /opt/ros/foxy/lib/librosidl_runtime_c.so
node: /opt/ros/foxy/lib/librcutils.so
node: CMakeFiles/node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/workspace/build/drone/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/node.dir/build: node

.PHONY : CMakeFiles/node.dir/build

CMakeFiles/node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/node.dir/clean

CMakeFiles/node.dir/depend:
	cd /root/workspace/build/drone && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/workspace/src/drone /root/workspace/src/drone /root/workspace/build/drone /root/workspace/build/drone /root/workspace/build/drone/CMakeFiles/node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/node.dir/depend

