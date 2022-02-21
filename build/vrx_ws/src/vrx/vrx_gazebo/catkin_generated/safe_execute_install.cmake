execute_process(COMMAND "/home/brad/catkin_ws/build/vrx_ws/src/vrx/vrx_gazebo/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/brad/catkin_ws/build/vrx_ws/src/vrx/vrx_gazebo/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
