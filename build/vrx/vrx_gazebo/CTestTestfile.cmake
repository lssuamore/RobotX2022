# CMake generated Testfile for 
# Source directory: /home/taylor/RobotX2022/src/vrx/vrx_gazebo
# Build directory: /home/taylor/RobotX2022/build/vrx/vrx_gazebo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_vrx_gazebo_rostest_test_sandisland.test "/home/taylor/RobotX2022/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/taylor/RobotX2022/build/test_results/vrx_gazebo/rostest-test_sandisland.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/taylor/RobotX2022/src/vrx/vrx_gazebo --package=vrx_gazebo --results-filename test_sandisland.xml --results-base-dir \"/home/taylor/RobotX2022/build/test_results\" /home/taylor/RobotX2022/src/vrx/vrx_gazebo/test/sandisland.test ")
set_tests_properties(_ctest_vrx_gazebo_rostest_test_sandisland.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/taylor/RobotX2022/src/vrx/vrx_gazebo/CMakeLists.txt;399;add_rostest_gtest;/home/taylor/RobotX2022/src/vrx/vrx_gazebo/CMakeLists.txt;0;")
subdirs("msgs")
