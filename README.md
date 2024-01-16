# SLAM - ITBdelabo
ITBdeLabo project to develop SLAM robot

# Install requires
1. Once you clone this repository to your workspace, go to the `install_requires` folder.
```
cd ~/catkin_ws/src/SLAM_ITBdeLabo/install_requires
```
2. Edit the permission of the executable file.
```
sudo chmod +x noetic_dep.sh
```
3. Run `noetic_dep.sh` script and install.
```
sudo ./noetic_dep.sh
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd ~/catkin_ws
catkin_make
```
4. Install msg & srv definition
```bash
cd ~/catkin_ws/src
git clone https://github.com/itbdelaboprogramming/ros_msd700_msgs.git
cd ~/catkin_ws
catkin_make
```
5. for slam_itbdelabo package, fill the config file from `config/slam.yaml`.
6. Refer to this [link](https://github.com/itbdelaboprogramming/firmware-msd700) for MSD700 microcontroller firmware.

# Preparation
1. Choose the local network of your machine that you want to connect. If you are using ZeroTier, find your robot machine IP address in ZeroTier network.
2. Connect to the mobile robot from a computer that has been connected to the network by entering this command into the computer's terminal.
```
ssh -X uname@192.168.xx.xx
```
Replace the username and the IP address that correspond to your machine configuration.

3. Then enter the mobile robot's password.

# Running program in Robot Machine
1. Connect the RPLidar USB cable to the USB port of your robot machine.
2. From the computer's terminal that has been connected to the mobile robot via ssh, enter this command to start the RPLidar scan.
```
roslaunch slam_itbdelabo irbot_rplidar.launch
```
If you are using other than RPLidar A1, please refer to your Lidar sensor page to run the sensor. If you are still using RPLidar product you can refer to this ref. ([RPLidar_ROS](https://github.com/Slamtec/rplidar_ros))

# Running program in Ubuntu computer
1. Prepare a computer that has Ubuntu operating system and ROS already installed. You can choose either to your local robot machine or other multiple machine to run RViz. If you are planning to use multiple configuration, you can follow the next step.
2. Connect the Ubuntu computer to the local network of the robot machine, then check the IP address of the computer that connects to the local network by entering this command in the Ubuntu terminal.
```
hostname -I
```
3. Edit the `.bashrc` file by entering this command in the Ubuntu terminal.
```
nano .bashrc
```
4. Add this line to the end of the file.
```
export ROS_IP = $LOCAL_IP
export ROS_HOSTNAME = $LOCAL_IP
export ROS_MASTER_URI = http://$ROBOT_IP:11311
```
5. Replace `$LOCAL_IP` with the IP shown in `hostname -I`. It should be some kind of this value `192.168.43.XX`.
6. Replace `$ROBOT_IP` with the robot IP address in the local network.
7. Step 3 to 6 are only needed to be done once.
8. Open the Ubuntu terminal then enter this command to run the SLAM program in RViz.
```
roslaunch slam_itbdelabo_rviz slam_hector.launch
```
