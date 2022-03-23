# LSSU/UniBz WAMV Codebase
## General Information
This is LSSU/UniBz WAMV Codebase for VRX and RobotX.  

## Installation Process:
Run the following. Things grouped together can be run together
```

#codebase download and install
cd ~
git clone https://github.com/lssuamore/RobotX2022.git
username:lssuamore
password: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

cd RobotX2022
git submodule init
git submodule update
delete devel and build folders
#note the submodule commands shuld install opencv and geonav_transform. Might also need to git clone PCL: https://github.com/ros-perception/perception_pcl.git
catkin_make
#after catkin_make update amore folder CMakelist(add_dependicies and target_link_libraries) by uncommenting the commented out programs


#dependencies
#note if CMake error occurs below:
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:75 (find_package):
  Could not find a package configuration file provided by ""
  with any of the following names:
#download the appropriate packages by finding the download on google 
usually in the form sudo apt install ros-noetic-""
