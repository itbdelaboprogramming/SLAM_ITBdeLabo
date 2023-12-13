#!/usr/bin/env python3

import rospy
from slam_itbdelabo.msg import HardwareCommand
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from slam_itbdelabo.srv import SetMapping, SetMappingResponse, SetMappingRequest
import subprocess
from datetime import datetime
import os, signal
import numpy as np
import math

# Global Variables
start_mapping = False
stop_mapping = False
pause_mapping = False

# Initialize ROS Node
rospy.init_node('slam_node')

# Get ROS Parameters (loaded from slam.yaml)
compute_period = rospy.get_param("/compute_period")
max_speed_linear = rospy.get_param("/max_speed_linear")
max_speed_angular = rospy.get_param("/max_speed_angular")
wheel_radius = rospy.get_param("/wheel_radius","2.75")		# in cm
wheel_distance = rospy.get_param("/wheel_distance","23.0")		# in cm
view_degrees = rospy.get_param("/view_degrees")
distance_threshold = rospy.get_param("/distance_threshold")
slam_folder_path = rospy.get_param("/slam_folder_path")

# Create ROS Publisher
hardware_command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)

# Create ROS Subscriber
vx = 0.0
wz = 0.0
def cmd_vel_callback(msg):
    global vx
    global wz
    vx = msg.linear.x
    wz = msg.angular.z
cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)


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

# Create ROS Service
def set_mapping_handler(req: SetMappingRequest):
    global start_mapping
    global pause_mapping
    global stop_mapping
    if sum([req.start, req.pause, req.stop]) > 1:
        success = False
        code = 2
    else:
        start_mapping = False
        pause_mapping = False
        stop_mapping = False
        success = True
        code = 0
        if req.start:
            start_mapping = True
        elif req.pause:
            pause_mapping = True
        elif req.stop:
            stop_mapping = True
        response = SetMappingResponse(success, code)
        return response
rospy.Service('set_mapping', SetMapping, set_mapping_handler)

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency)

while not rospy.is_shutdown():
    hardware_command_msg = HardwareCommand()
    
    if start_mapping == 1 :
    	# inverse kinematics
        vx = constrain(vx, -max_speed_linear, max_speed_linear)
        wz = constrain(wz, -max_speed_angular, max_speed_angular)
        hardware_command_msg.right_motor_speed = (vx*100.0/wheel_radius - wz*wheel_distance/(2.0*wheel_radius))*9.55
        hardware_command_msg.left_motor_speed = (vx*100.0/wheel_radius + wz*wheel_distance/(2.0*wheel_radius))*9.55
    else:
        hardware_command_msg.right_motor_speed = 0.0
        hardware_command_msg.left_motor_speed = 0.0

    
    # convention, rot_vel (+) -> clockwise (navigation/compass-based)
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
    
    rate.sleep()
