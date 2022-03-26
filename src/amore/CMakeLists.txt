cmake_minimum_required(VERSION 3.0.2)
project(amore)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
  geometry_msgs
  geographic_msgs
  cv_bridge
  rospy
  nav_msgs 
  sensor_msgs
  image_transport
  pcl_ros
  pcl_conversions
  )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NED_waypoints.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amore
   CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/amore.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(PID_sub src/PID_sub.cpp)
add_executable(PID_HP_control src/PID_HP_control.cpp)
add_executable(Station_keeping src/Station_keeping.cpp)
add_executable(gps_subscriber src/gps_subscriber.cpp)
add_executable(Visions_Path_Planner src/Visions_Path_Planner.cpp)
add_executable(camera_subscriber src/camera_subscriber.cpp)
add_executable(High_Level_trajectory_processor src/High_Level_trajectory_processor.cpp)
add_executable(traj_pub src/traj_pub.cpp)

add_executable(color_test src/color_test.cpp)
add_executable(object_d src/object_d.cpp)
add_executable(stereo_vis src/stereo_vis.cpp)
add_executable(gps_imu src/gps_imu.cpp)
add_executable(camera_lidar src/camera_lidar.cpp)
add_executable(lidar_subscriber src/lidar_subscriber.cpp)
add_executable(SK_Controller src/SK_Controller.cpp)
add_executable(new_gps_imu src/new_gps_imu.cpp)
add_executable(mission_control src/mission_control.cpp)

# TASK 1: STATION-KEEPING
add_executable(Task1_SK_point_converter src/Task1_SK_point_converter.cpp)
add_executable(Task1_SK_Planner src/Task1_SK_Planner.cpp)

# TASK 2: WAYFINDING
add_executable(Task2_WF_gps_imu src/Task2_WF_gps_imu.cpp)
add_executable(Task2_WF_waypoints_converter src/Task2_WF_waypoints_converter.cpp)
add_executable(Task2_WF_Planner src/Task2_WF_Planner.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(PID_sub ${PID_sub_EXPORTED_TARGETS} {catkin_EXPORTED_TARGETS})
#add_dependencies(PID_pub ${PID_pub_EXPORTED_TARGETS} {catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(PID_sub ${catkin_LIBRARIES})
target_link_libraries(PID_HP_control ${catkin_LIBRARIES})
target_link_libraries(Station_keeping ${catkin_LIBRARIES})
target_link_libraries(gps_subscriber ${catkin_LIBRARIES})
target_link_libraries(Visions_Path_Planner ${catkin_LIBRARIES})
target_link_libraries(camera_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(High_Level_trajectory_processor ${catkin_LIBRARIES})
target_link_libraries(traj_pub ${catkin_LIBRARIES})

target_link_libraries(color_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(object_d ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(stereo_vis ${catkin_LIBRARIES})
target_link_libraries(gps_imu ${catkin_LIBRARIES})
target_link_libraries(camera_lidar ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS})
target_link_libraries(lidar_subscriber ${catkin_LIBRARIES} ${PCL_LIBS})
target_link_libraries(SK_Controller ${catkin_LIBRARIES})
target_link_libraries(new_gps_imu ${catkin_LIBRARIES})
target_link_libraries(mission_control ${catkin_LIBRARIES})

# TASK 1: STATION-KEEPING
target_link_libraries(Task1_SK_point_converter ${catkin_LIBRARIES})
target_link_libraries(Task1_SK_Planner ${catkin_LIBRARIES})

# TASK 2: WAYFINDING
target_link_libraries(Task2_WF_gps_imu ${catkin_LIBRARIES})
target_link_libraries(Task2_WF_waypoints_converter ${catkin_LIBRARIES})
target_link_libraries(Task2_WF_Planner ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES launch/Visions_SSP_BH.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES launch/SK.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES launch/Task2_WF.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES launch/traj_track.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES launch/Task3.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES launch/VRX_AMORE.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_amore.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
