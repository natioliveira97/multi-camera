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
CMAKE_SOURCE_DIR = /home/natalia/tg_workspace/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/natalia/tg_workspace/catkin_ws/build

# Include any dependencies generated for this target.
include openni_camera/openni_camera/CMakeFiles/openni_node.dir/depend.make

# Include the progress variables for this target.
include openni_camera/openni_camera/CMakeFiles/openni_node.dir/progress.make

# Include the compile flags for this target's objects.
include openni_camera/openni_camera/CMakeFiles/openni_node.dir/flags.make

openni_camera/openni_camera/CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.o: openni_camera/openni_camera/CMakeFiles/openni_node.dir/flags.make
openni_camera/openni_camera/CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.o: /home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/src/nodes/openni_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/natalia/tg_workspace/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object openni_camera/openni_camera/CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.o"
	cd /home/natalia/tg_workspace/catkin_ws/build/openni_camera/openni_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.o -c /home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/src/nodes/openni_node.cpp

openni_camera/openni_camera/CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.i"
	cd /home/natalia/tg_workspace/catkin_ws/build/openni_camera/openni_camera && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/src/nodes/openni_node.cpp > CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.i

openni_camera/openni_camera/CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.s"
	cd /home/natalia/tg_workspace/catkin_ws/build/openni_camera/openni_camera && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera/src/nodes/openni_node.cpp -o CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.s

# Object files for target openni_node
openni_node_OBJECTS = \
"CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.o"

# External object files for target openni_node
openni_node_EXTERNAL_OBJECTS =

/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: openni_camera/openni_camera/CMakeFiles/openni_node.dir/src/nodes/openni_node.cpp.o
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: openni_camera/openni_camera/CMakeFiles/openni_node.dir/build.make
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /home/natalia/tg_workspace/catkin_ws/devel/lib/libopenni_driver.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libcamera_info_manager.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libcamera_calibration_parsers.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libimage_transport.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libmessage_filters.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libnodeletlib.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libbondcpp.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libclass_loader.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libroslib.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/librospack.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libroscpp.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/librosconsole.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/librostime.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libcpp_common.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/librostime.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /opt/ros/noetic/lib/libcpp_common.so
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node: openni_camera/openni_camera/CMakeFiles/openni_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/natalia/tg_workspace/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node"
	cd /home/natalia/tg_workspace/catkin_ws/build/openni_camera/openni_camera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openni_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
openni_camera/openni_camera/CMakeFiles/openni_node.dir/build: /home/natalia/tg_workspace/catkin_ws/devel/lib/openni_camera/openni_node

.PHONY : openni_camera/openni_camera/CMakeFiles/openni_node.dir/build

openni_camera/openni_camera/CMakeFiles/openni_node.dir/clean:
	cd /home/natalia/tg_workspace/catkin_ws/build/openni_camera/openni_camera && $(CMAKE_COMMAND) -P CMakeFiles/openni_node.dir/cmake_clean.cmake
.PHONY : openni_camera/openni_camera/CMakeFiles/openni_node.dir/clean

openni_camera/openni_camera/CMakeFiles/openni_node.dir/depend:
	cd /home/natalia/tg_workspace/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/natalia/tg_workspace/catkin_ws/src /home/natalia/tg_workspace/catkin_ws/src/openni_camera/openni_camera /home/natalia/tg_workspace/catkin_ws/build /home/natalia/tg_workspace/catkin_ws/build/openni_camera/openni_camera /home/natalia/tg_workspace/catkin_ws/build/openni_camera/openni_camera/CMakeFiles/openni_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : openni_camera/openni_camera/CMakeFiles/openni_node.dir/depend

