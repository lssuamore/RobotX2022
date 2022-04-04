# LSSU/UniBz WAMV Codebase
## General Information
This is LSSU/UniBz WAMV Codebase for VRX and RobotX. Brad loves ☕.  

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
