#!/bin/bash

sudo apt dist-upgrade
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source .bashrc
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-rqt-*
sudo apt install ros-humble-joy*

pip install --upgrade pip

pip install customtkinter
pip install ultralytics
pip install opencv-python
pip install pyrealsense2
pip install python-can