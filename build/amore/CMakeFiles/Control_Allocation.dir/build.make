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
CMAKE_SOURCE_DIR = /home/brad/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/brad/catkin_ws/build

# Include any dependencies generated for this target.
include amore/CMakeFiles/Control_Allocation.dir/depend.make

# Include the progress variables for this target.
include amore/CMakeFiles/Control_Allocation.dir/progress.make

# Include the compile flags for this target's objects.
include amore/CMakeFiles/Control_Allocation.dir/flags.make

amore/CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.o: amore/CMakeFiles/Control_Allocation.dir/flags.make
amore/CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.o: /home/brad/catkin_ws/src/amore/src/Control_Allocation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brad/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object amore/CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.o"
	cd /home/brad/catkin_ws/build/amore && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.o -c /home/brad/catkin_ws/src/amore/src/Control_Allocation.cpp

amore/CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.i"
	cd /home/brad/catkin_ws/build/amore && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brad/catkin_ws/src/amore/src/Control_Allocation.cpp > CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.i

amore/CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.s"
	cd /home/brad/catkin_ws/build/amore && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brad/catkin_ws/src/amore/src/Control_Allocation.cpp -o CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.s

# Object files for target Control_Allocation
Control_Allocation_OBJECTS = \
"CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.o"

# External object files for target Control_Allocation
Control_Allocation_EXTERNAL_OBJECTS =

/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: amore/CMakeFiles/Control_Allocation.dir/src/Control_Allocation.cpp.o
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: amore/CMakeFiles/Control_Allocation.dir/build.make
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/libroscpp.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/librosconsole.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/librostime.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /opt/ros/noetic/lib/libcpp_common.so
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/brad/catkin_ws/devel/lib/amore/Control_Allocation: amore/CMakeFiles/Control_Allocation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/brad/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/brad/catkin_ws/devel/lib/amore/Control_Allocation"
	cd /home/brad/catkin_ws/build/amore && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Control_Allocation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
amore/CMakeFiles/Control_Allocation.dir/build: /home/brad/catkin_ws/devel/lib/amore/Control_Allocation

.PHONY : amore/CMakeFiles/Control_Allocation.dir/build

amore/CMakeFiles/Control_Allocation.dir/clean:
	cd /home/brad/catkin_ws/build/amore && $(CMAKE_COMMAND) -P CMakeFiles/Control_Allocation.dir/cmake_clean.cmake
.PHONY : amore/CMakeFiles/Control_Allocation.dir/clean

amore/CMakeFiles/Control_Allocation.dir/depend:
	cd /home/brad/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brad/catkin_ws/src /home/brad/catkin_ws/src/amore /home/brad/catkin_ws/build /home/brad/catkin_ws/build/amore /home/brad/catkin_ws/build/amore/CMakeFiles/Control_Allocation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : amore/CMakeFiles/Control_Allocation.dir/depend

