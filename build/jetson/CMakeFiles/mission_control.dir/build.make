# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/amore/RobotX2022/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amore/RobotX2022/build

# Include any dependencies generated for this target.
include jetson/CMakeFiles/mission_control.dir/depend.make

# Include the progress variables for this target.
include jetson/CMakeFiles/mission_control.dir/progress.make

# Include the compile flags for this target's objects.
include jetson/CMakeFiles/mission_control.dir/flags.make

jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o: jetson/CMakeFiles/mission_control.dir/flags.make
jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o: /home/amore/RobotX2022/src/jetson/src/mission_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amore/RobotX2022/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o"
	cd /home/amore/RobotX2022/build/jetson && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mission_control.dir/src/mission_control.cpp.o -c /home/amore/RobotX2022/src/jetson/src/mission_control.cpp

jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mission_control.dir/src/mission_control.cpp.i"
	cd /home/amore/RobotX2022/build/jetson && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amore/RobotX2022/src/jetson/src/mission_control.cpp > CMakeFiles/mission_control.dir/src/mission_control.cpp.i

jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mission_control.dir/src/mission_control.cpp.s"
	cd /home/amore/RobotX2022/build/jetson && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amore/RobotX2022/src/jetson/src/mission_control.cpp -o CMakeFiles/mission_control.dir/src/mission_control.cpp.s

jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.requires:

.PHONY : jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.requires

jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.provides: jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.requires
	$(MAKE) -f jetson/CMakeFiles/mission_control.dir/build.make jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.provides.build
.PHONY : jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.provides

jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.provides.build: jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o


# Object files for target mission_control
mission_control_OBJECTS = \
"CMakeFiles/mission_control.dir/src/mission_control.cpp.o"

# External object files for target mission_control
mission_control_EXTERNAL_OBJECTS =

/home/amore/RobotX2022/devel/lib/jetson/mission_control: jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o
/home/amore/RobotX2022/devel/lib/jetson/mission_control: jetson/CMakeFiles/mission_control.dir/build.make
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libcv_bridge.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libimage_transport.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libmessage_filters.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libclass_loader.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/libPocoFoundation.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libdl.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libroscpp.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/librosconsole.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libroslib.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/librospack.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/librostime.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /opt/ros/melodic/lib/libcpp_common.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/amore/RobotX2022/devel/lib/jetson/mission_control: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/amore/RobotX2022/devel/lib/jetson/mission_control: jetson/CMakeFiles/mission_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amore/RobotX2022/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/amore/RobotX2022/devel/lib/jetson/mission_control"
	cd /home/amore/RobotX2022/build/jetson && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mission_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
jetson/CMakeFiles/mission_control.dir/build: /home/amore/RobotX2022/devel/lib/jetson/mission_control

.PHONY : jetson/CMakeFiles/mission_control.dir/build

jetson/CMakeFiles/mission_control.dir/requires: jetson/CMakeFiles/mission_control.dir/src/mission_control.cpp.o.requires

.PHONY : jetson/CMakeFiles/mission_control.dir/requires

jetson/CMakeFiles/mission_control.dir/clean:
	cd /home/amore/RobotX2022/build/jetson && $(CMAKE_COMMAND) -P CMakeFiles/mission_control.dir/cmake_clean.cmake
.PHONY : jetson/CMakeFiles/mission_control.dir/clean

jetson/CMakeFiles/mission_control.dir/depend:
	cd /home/amore/RobotX2022/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amore/RobotX2022/src /home/amore/RobotX2022/src/jetson /home/amore/RobotX2022/build /home/amore/RobotX2022/build/jetson /home/amore/RobotX2022/build/jetson/CMakeFiles/mission_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : jetson/CMakeFiles/mission_control.dir/depend
