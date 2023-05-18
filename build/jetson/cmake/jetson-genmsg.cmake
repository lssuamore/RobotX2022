# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "jetson: 10 messages, 0 services")

set(MSG_I_FLAGS "-Ijetson:/home/amore/RobotX2022/src/jetson/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(jetson_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg" "std_msgs/Header:geometry_msgs/Point:geometry_msgs/PointStamped"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg" "std_msgs/Header:geometry_msgs/Point:geometry_msgs/PointStamped"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg" "std_msgs/Float32"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg" "std_msgs/Header:geometry_msgs/Point:geometry_msgs/PointStamped"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg" ""
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/task_info.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/task_info.msg" "std_msgs/Int64"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg" "std_msgs/Float64:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg" "std_msgs/Int32"
)

get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg" NAME_WE)
add_custom_target(_jetson_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jetson" "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg" "std_msgs/Int32:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/task_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)
_generate_msg_cpp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
)

### Generating Services

### Generating Module File
_generate_module_cpp(jetson
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(jetson_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(jetson_generate_messages jetson_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/task_info.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_cpp _jetson_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetson_gencpp)
add_dependencies(jetson_gencpp jetson_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetson_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/task_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)
_generate_msg_eus(jetson
  "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
)

### Generating Services

### Generating Module File
_generate_module_eus(jetson
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(jetson_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(jetson_generate_messages jetson_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/task_info.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_eus _jetson_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetson_geneus)
add_dependencies(jetson_geneus jetson_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetson_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/task_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)
_generate_msg_lisp(jetson
  "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
)

### Generating Services

### Generating Module File
_generate_module_lisp(jetson
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(jetson_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(jetson_generate_messages jetson_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/task_info.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_lisp _jetson_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetson_genlisp)
add_dependencies(jetson_genlisp jetson_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetson_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/task_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)
_generate_msg_nodejs(jetson
  "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
)

### Generating Services

### Generating Module File
_generate_module_nodejs(jetson
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(jetson_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(jetson_generate_messages jetson_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/task_info.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_nodejs _jetson_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetson_gennodejs)
add_dependencies(jetson_gennodejs jetson_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetson_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/task_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)
_generate_msg_py(jetson
  "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
)

### Generating Services

### Generating Module File
_generate_module_py(jetson
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(jetson_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(jetson_generate_messages jetson_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_objects.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/zed2i_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/NED_waypoints.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/control_efforts.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Detect_Dock_Fling.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/AMS_state.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/task_info.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/usv_pose_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/Acoustics_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amore/RobotX2022/src/jetson/msg/state_msg.msg" NAME_WE)
add_dependencies(jetson_generate_messages_py _jetson_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jetson_genpy)
add_dependencies(jetson_genpy jetson_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jetson_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jetson
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(jetson_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(jetson_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jetson
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(jetson_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(jetson_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jetson
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(jetson_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(jetson_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jetson
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(jetson_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(jetson_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jetson
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(jetson_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(jetson_generate_messages_py std_msgs_generate_messages_py)
endif()
