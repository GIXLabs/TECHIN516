#!/bin/bash

# update before installing anything
sudo apt update

source /opt/ros/humble/setup.bash

# install python packages
sudo apt install -y python3-pip
pip install matplotlib

# install helpful tools
sudo apt install -y \
    ros-humble-tf2-tools \
    ros-humble-rqt-* \
    net-tools

# install gazebo
sudo apt install -y \
    ros-humble-gazebo-*
echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc

# install kinova dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-colcon-clean \
    python3-vcstool \
    ros-humble-moveit
pip install moveit_config_utils
pip install pandas
pip install pyquaternion

# create src directory
mkdir src

# install kinova packages
git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

# install pymoveit2
cd src
git clone https://github.com/AndrejOrsula/pymoveit2.git
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths ./pymoveit2
cd ..

rosdep install --ignore-src --from-paths src -y -r
colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
source install/setup.bash