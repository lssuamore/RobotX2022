# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;cmake_modules;geographic_msgs;geometry_msgs;nav_msgs;sensor_msgs;std_msgs;tf2;tf2_geometry_msgs;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgeonav_transform".split(';') if "-lgeonav_transform" != "" else []
PROJECT_NAME = "geonav_transform"
PROJECT_SPACE_DIR = "/home/taylor/RobotX2022/install"
PROJECT_VERSION = "0.0.1"
