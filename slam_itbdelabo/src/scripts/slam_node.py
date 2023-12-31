#!/usr/bin/env python3

import rospy
from slam_itbdelabo.msg import HardwareCommand
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import subprocess
import os, signal
import numpy as np
import math
import paho.mqtt.client as mqtt
import random
import json

# Global Variables
start_mapping = False
stop_mapping = False
pause_mapping = False
compute_slam_process = None
vx = 0.0
wz = 0.0

# Initialize ROS Node
rospy.init_node('slam_node')

# Get ROS Parameters (loaded from slam.yaml)
compute_period = rospy.get_param("/slam_node/compute_period")
max_speed_linear = rospy.get_param("/slam_node/max_speed_linear")
max_speed_angular = rospy.get_param("/slam_node/max_speed_angular")
wheel_radius = rospy.get_param("/slam_node/wheel_radius")		# in cm
wheel_distance = rospy.get_param("/slam_node/wheel_distance")		# in cm
view_degrees = rospy.get_param("/slam_node/view_degrees")
distance_threshold = rospy.get_param("/slam_node/distance_threshold")
mqtt_broker_ip = rospy.get_param("/slam_node/mqtt_broker_ip")
use_simulator = bool(rospy.get_param("/slam_node/use_simulator"))
model_name = rospy.get_param("/slam_node/model_name")

# Create ROS Publisher
hardware_command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def Angle2Index(laser_scan_msg, angle):
    return (int)((angle-laser_scan_msg.angle_min)/laser_scan_msg.angle_increment)
def Index2Angle(laser_scan_msg, index):
    return (laser_scan_msg.angle_min + (index*laser_scan_msg.angle_increment))
def scan_callback(msg: LaserScan):
    global vx
    global wz
    global view_degrees
    global distance_threshold
    # range in m, angle in rad
    min_view_deg = -view_degrees # deg
    max_view_deg = view_degrees  # deg
    min_view_idx = Angle2Index(msg, math.radians(min_view_deg))
    center_view_idx = Angle2Index(msg, math.radians(0))
    max_view_idx = Angle2Index(msg, math.radians(max_view_deg))

    arr_range = msg.ranges
    right_arr = arr_range[min_view_idx:center_view_idx]
    left_arr = arr_range[center_view_idx:max_view_idx+1]
    right_filtered_arr = [x for x in right_arr if not math.isnan(x) and not math.isinf(x)]
    left_filtered_arr = [x for x in left_arr if not math.isnan(x) and not math.isinf(x)]
    np_right_filtered = np.array(right_filtered_arr)
    np_left_filtered = np.array(left_filtered_arr)
    right_obs = np.any(np_right_filtered <= distance_threshold)
    left_obs = np.any(np_left_filtered <= distance_threshold)
    if left_obs:
        vx = 0.0
        wz = 1.0
    elif right_obs:
        vx = 0.0
        wz = -1.0
    else:
        vx = 0.10
        wz = 0.0
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)

# MQTT Set Up
def on_connect(client, userdata, flags, rc):
    print(f"Connected with MQTT Broker at {mqtt_broker_ip}")
    client.subscribe("/mapping")
    client.subscribe("/lidar")

def on_message(client, userdata, msg):
    global start_mapping
    global pause_mapping
    global stop_mapping
    global compute_slam_process
    if msg.topic == "/mapping":
        payload = int(msg.payload)
        if payload == 1:
            start_mapping = True
            pause_mapping = False
            stop_mapping = False
        elif payload == 2:
            start_mapping = False
            pause_mapping = True
            stop_mapping = False
        elif payload == 3:
            start_mapping = False
            pause_mapping = False
            stop_mapping = True
        else:
            start_mapping = False
            pause_mapping = False
            stop_mapping = False
    elif msg.topic == "/lidar":
        data = json.loads(msg.payload.decode("utf-8"))
        enable = bool(data["enable"])
        use_own_map = bool(data["use_own_map"])
        if enable and compute_slam_process is None:
            if use_own_map:
                print("Launching compute_slam with own map")
                compute_slam_process = subprocess.Popen(["roslaunch", "slam_itbdelabo", "compute_slam.launch", "use_own_map:=true"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            else:
                print("Launching compute slam")
                compute_slam_process = subprocess.Popen(["roslaunch", "slam_itbdelabo", "compute_slam.launch", f"use_simulator:={use_simulator}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif not enable and compute_slam_process is not None:
            os.killpg(os.getpgid(compute_slam_process.pid), signal.SIGTERM)
        
mqtt_client = mqtt.Client(client_id=f"slam_node_${random.randint(0,1000)}")
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(mqtt_broker_ip, 1883, 60)

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency)

while not rospy.is_shutdown():
    hardware_command_msg = HardwareCommand()
    twist_msg = Twist()
    # print(f"start_mapping: {start_mapping}, pause_mapping: {pause_mapping}, stop_mapping: {stop_mapping}")
    if start_mapping:
    	# inverse kinematics
        vx = constrain(vx, -max_speed_linear, max_speed_linear)
        wz = constrain(wz, -max_speed_angular, max_speed_angular)
        hardware_command_msg.right_motor_speed = (vx*100.0/wheel_radius - wz*wheel_distance/(2.0*wheel_radius))*9.55
        hardware_command_msg.left_motor_speed = (vx*100.0/wheel_radius + wz*wheel_distance/(2.0*wheel_radius))*9.55
        twist_msg.linear.x = vx
        twist_msg.angular.z = wz
    else:
        hardware_command_msg.right_motor_speed = 0.0
        hardware_command_msg.left_motor_speed = 0.0
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

    
    # convention, rot_vel (+) -> clockwise (navigation/compass-based)
    if start_mapping:
        if vx > 0 :
            # forward
            hardware_command_msg.movement_command = 3
        elif wz < 0:
            # left
            hardware_command_msg.movement_command = 2 
        elif wz > 0:
            # right
            hardware_command_msg.movement_command = 1 
    else:
        # stop
        hardware_command_msg.movement_command = 0
    
    hardware_command_pub.publish(hardware_command_msg)
    cmd_vel_pub.publish(twist_msg)
    mqtt_client.loop(timeout=compute_period/1000)
    
    rate.sleep()
