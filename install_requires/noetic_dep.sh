#!/bin/bash

echo "[Install ROS Dependencies]"
sudo apt-get install -y ros-noetic-rosserial-arduino ros-noetic-rosserial
sudo apt-get install -y ros-noetic-rplidar-ros ros-noetic-teleop-twist-keyboard
sudo apt-get install -y ros-noetic-slam-karto ros-noetic-hector-slam ros-noetic-gmapping
sudo apt-get install -y ros-noetic-move-base ros-noetic-dwa-local-planner ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-topic-tools ros-noetic-explore-lite ros-noetic-tf2-tools
sudo apt-get install -y ros-noetic-map-server
sudo apt-get install -y ros-noetic-rosbridge-server