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
delete devel and build folders
#note you will need to git clone all the empty folders like opencv and geonav_transform. Might also need to git clone PCL
catkin_make

#dependencies
sudo apt-get install libpcap-dev
sudo apt install ros-noetic-geographic-msgs
sudo apt install ros-noetic-costmap-2d
sudo apt install ros-noetic-move-base-msgs
sudo apt install ros-noetic-robot-localization
sudo apt install ros-noetic-pointcloud-to-laserscan


#OpenCV install
mkdir opencv
git clone https://github.com/opencv/opencv_contrib.git
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \-D CMAKE_INSTALL_PREFIX=/usr/local \-D INSTALL_C_EXAMPLES=OFF \-D INSTALL_PYTHON_EXAMPLES=OFF \-D WITH_TBB=ON \-D WITH_V4L=ON \-D WITH_QT=ON \-D WITH_OPENGL=ON \-D WITH_CUDA=ON \-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \-D BUILD_EXAMPLES=OFF ..
make
sudo make install
cd ~
