# LSSU/UniBz WAMV Codebase
## General Information
This is LSSU/UniBz WAMV Codebase for VRX and RobotX.  

## Installation Process:
Run the following.
```

#codebase download and install
cd ~
git clone https://github.com/lssuamore/RobotX2022.git
username:lssuamore
password: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

cd RobotX2022
git submodule update --init --recursive
git submodule init
git submodule update
delete devel and build folders
#note the submodule commands shuld install opencv and geonav_transform. Might also need to git clone PCL: https://github.com/ros-perception/perception_pcl.git
#if submodule commands dont work delete empty folders and in there place install the correct packages from github
catkin_make
#after catkin_make update amore folder CMakelist(add_dependicies and target_link_libraries) by uncommenting the commented out programs


#dependencies
#note if CMake error occurs below:
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:75 (find_package):
  Could not find a package configuration file provided by ""
  with any of the following names:
#download the appropriate packages by finding the download on google 
usually in the form sudo apt install ros-noetic-""



Git Commands

git checkout -b <your name>
git checkout is how you will switch branches. The -b indicates we are creating a new branch. The branch name should be an indication on what you are working on. In this case it is adding your name to the list so it is your own name.

git add -A
This command adds all changes to be tracked that you changed. The -A parameter is what specificlly sets everything. You can also use git rm and other similar commands. As always, see the git --help command for specifics pas the tutorial.

git commit -m "Your message here"
The commit command does what it sounds like. You are commiting to the code as it should have made some change. This can be as simple as finishing up for the night though, it doesn't need to be cohesive. This is not pushing any code github yet, this is just locally saved on your computer.

git push orgin <your name>
The push command sends your code to the github servers. orgin just indicates that it is going where we got it from, no real need to work with this in what we develop with. Last you have to make sure you choose your branch to send it too. Again in this case the branch is your name, but if you ever forget what it is called, you can always use the git status or git branch command.

Pull requests:
Now that you have developed all of this code (and tested), you would like it to be included in the master distribution. This is done using a pull request, and whoever maintins the repository will manage any conflict issues. This is done by going to the repository itself, and clicking the pull request tab as seen in the picture below. PullRequest.
