# LSSU/UniBz WAMV Codebase
## General Information
This repository holds all of the codes and necessary packages for Team AMORE's RobotX control software solution. The setup of the code uses one ROS .launch file to launch the geonav_transform package, and 5 .cpp executables. The geonav_transform package is used for converting ECEF latitude longitude coordinates to a local NED x-y plane to work on. The 5 .cpp executables include mission_control, path_planner, navigation_array, perception_array, and propulsion_system. The mission_control monitors the state of the current task and sets the states of the other executables based on the overall system status. path_planner was designed to handle computing trajectories and feeding them to the propulsion_system. navigation_array publishes ECEF states to nav_odom so that geonav_transform is able to subscribe and publish its conversion to a local ENU frame to "geonav_odom". navigation_array then converts the ENU pose to a NED pose. perception_array will use open source packages for the ZED 2i and the 16 beam Velodyne LiDAR Puck to perform object localization and characterization. The propulsion_system is designed to handle getting the goal pose and calculating the desired control efforts and using allocation to convert those efforts to outputs on the Teensy microcontrollers. The control efforts are calculated using PID control theory to control heading and either position or speed. Current available drive configurations include dual-azimuthing, differential, and Ackermann. Dual-azimuthing is used for station-keeping and the other two are used for wayfinding.

## Update repository with master
```
cd RobotX2022/
```
Make sure you are on the master branch with the following:
```
git checkout master
```
Pull the repository to your computer with the following:
```
git pull
```
username:
```
lssuamore
```
password:
```
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
```
## Installation Process:

```
cd ~
```
```
git clone https://github.com/lssuamore/RobotX2022.git
```
username:lssuamore    password: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
```
cd RobotX2022
```
Initialize and update submodules. The submodule commands should install opencv and geonav_transform. Might also need to git clone PCL: https://github.com/ros-perception/perception_pcl.git If submodule commands don't work, delete empty folders and git clone the correct packages from github in their place.
```
git submodule init
```
```
git submodule update
```
Delete the devel and build folders
```
catkin_make
```
After catkin_make, update amore folder CMakelist by uncommenting the commented out programs (add_executable and target_link_libraries). If CMake error below occurs:
```
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:75 (find_package):
  Could not find a package configuration file provided by ""
  with any of the following names:
```
Download the appropriate packages by finding the download on Google usually in the form:
```
sudo apt install ros-noetic-""
```
Download the following libraries if necessary...
```
sudo apt-get install ros-noetic-xacro
sudo apt-get install ros-noetic-gazebo_ros
sudo apt-get install ros-noetic-geographic-msgs
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-image-transport
sudo apt-get install ros-noetic-pcl-ros
sudo apt-get install ros-noetic-tf2-geometry-msgs
```

# Git Commands
git checkout is how you will switch branches. The -b indicates we are creating a new branch.
```
git checkout -b <your name>
```
The following command adds all changes to be tracked that you changed. The -A parameter is what specificlly sets everything. You can also use git rm and other similar commands. As always, see the git --help command for specifics past the tutorial.
```
git add -A
```
The commit command does what it sounds like. You are commiting to the code as it should have made some change. This can be as simple as finishing up for the night though, it doesn't need to be cohesive. This is not pushing any code github yet, this is just locally saved on your computer.
```
git commit -m "Your message here"
```
The push command sends your code to the github servers. origin just indicates that it is going where we got it from, no real need to work with this in what we develop with. Last you have to make sure you choose your branch to send it too. Again in this case the branch is your name, but if you ever forget what it is called, you can always use the git status or git branch command.
```
git push origin <your name>
```

Pull requests:
Now that you have developed all of this code (and tested), you would like it to be included in the master distribution. This is done using a pull request, and whoever maintains the repository will manage any conflicting issues. This is done by going to the repository itself, and clicking the pull request tab.
