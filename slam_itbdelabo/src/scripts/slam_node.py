#!/usr/bin/env python3

# Import Python Libraries
import rospy
import cv2
import tf
from ros_msd700_msgs.msg import HardwareCommand, HardwareState
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import subprocess
import os, signal
import numpy as np
import math
import paho.mqtt.client as mqtt
import random
import json

# Initialize ROS Node
rospy.init_node('slam_node')

# Get ROS Parameters (loaded from slam.yaml)
compute_period = rospy.get_param("/slam_node/compute_period", 20)
max_speed_linear = rospy.get_param("/slam_node/max_speed_linear", 0.33)
max_speed_angular = rospy.get_param("/slam_node/max_speed_angular", 1.75)
wheel_radius = rospy.get_param("/slam_node/wheel_radius", 2.75)	  # in cm
wheel_distance = rospy.get_param("/slam_node/wheel_distance", 23.0)    # in cm
gear_ratio = rospy.get_param("/slam_node/gear_ratio", 1980.0)
use_imu = rospy.get_param("/slam_node/use_imu", 0)
view_degrees = rospy.get_param("/slam_node/view_degrees", 90.0)
distance_threshold = rospy.get_param("/slam_node/distance_threshold", 0.7)
camera_id = rospy.get_param("/slam_node/camera_id")
camera_width = rospy.get_param("/slam_node/camera_width")
camera_height = rospy.get_param("/slam_node/camera_height")
camera_quality = rospy.get_param("/slam_node/camera_quality")
username = rospy.get_param("/slam_node/username","itbdelabo")
unit_name = rospy.get_param("/slam_node/unit_name","Unit A")
mqtt_broker_ip = rospy.get_param("/slam_node/mqtt_broker_ip")
use_simulator = bool(rospy.get_param("/slam_node/use_simulator"))
model_name = rospy.get_param("/slam_node/model_name","") 

# Global Variables
ch_ultrasonic_distance_1 = 0.0
ch_ultrasonic_distance_2 = 0.0
ch_ultrasonic_distance_3 = 0.0
ch_ultrasonic_distance_4 = 0.0
ch_ultrasonic_distance_5 = 0.0
ch_ultrasonic_distance_6 = 0.0
ch_ultrasonic_distance_7 = 0.0
ch_ultrasonic_distance_8 = 0.0
ch_ultrasonic_distances = []
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0
mapping_topic = f"{username}/{unit_name}/mapping"
lidar_topic = f"{username}/{unit_name}/lidar"
camera_topic = f"{username}/{unit_name}/camera"
start_mapping = False
stop_mapping = False
pause_mapping = False
start_navigation = False
compute_slam_process = None
compute_nav_process = None
vx = 0.0
wz = 0.0
pose_x = 0.0
pose_y = 0.0
theta = 0.0
heading = 0.0

# Create ROS Publishers
hardware_command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
odom_pub = rospy.Publisher('wheel/odom', Odometry, queue_size=1)

def warpAngle(angle):
    if angle >= 6.28318531:
        return warpAngle(angle - 6.28318531)
    elif angle < 0:
        return warpAngle(angle + 6.28318531)
    else:
        return angle

# Create ROS Subscribers
def hardware_state_callback(msg: HardwareState):
    global ch_ultrasonic_distance_1, ch_ultrasonic_distance_2, ch_ultrasonic_distance_3, ch_ultrasonic_distance_4, \
    ch_ultrasonic_distance_5, ch_ultrasonic_distance_6, ch_ultrasonic_distance_7, ch_ultrasonic_distance_8, \
    ch_ultrasonic_distances, right_motor_pulse_delta, left_motor_pulse_delta
    ch_ultrasonic_distance_1 = msg.ch_ultrasonic_distance_1
    ch_ultrasonic_distance_2 = msg.ch_ultrasonic_distance_2
    ch_ultrasonic_distance_3 = msg.ch_ultrasonic_distance_3
    ch_ultrasonic_distance_4 = msg.ch_ultrasonic_distance_4
    ch_ultrasonic_distance_5 = msg.ch_ultrasonic_distance_5
    ch_ultrasonic_distance_6 = msg.ch_ultrasonic_distance_6
    ch_ultrasonic_distance_7 = msg.ch_ultrasonic_distance_7
    ch_ultrasonic_distance_8 = msg.ch_ultrasonic_distance_8
    ch_ultrasonic_distances = [ch_ultrasonic_distance_1, ch_ultrasonic_distance_2, 
                               ch_ultrasonic_distance_3, ch_ultrasonic_distance_4, 
                               ch_ultrasonic_distance_5, ch_ultrasonic_distance_6, 
                               ch_ultrasonic_distance_7, ch_ultrasonic_distance_8]
    right_motor_pulse_delta = msg.right_motor_pulse_delta
    left_motor_pulse_delta = msg.left_motor_pulse_delta
    heading = msg.heading
hardware_state_sub = rospy.Subscriber("hardware_state", HardwareState, hardware_state_callback)

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
    if start_mapping and not start_navigation:
        if left_obs:
            vx = 0.0
            wz = 1.0
        elif right_obs:
            vx = 0.0
            wz = -1.0
        else:
            vx = 0.10
            wz = 0.0
    elif not start_mapping and not start_navigation:
        vx = 0.0
        wz = 0.0
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)

def cmd_vel_callback(msg: Twist):
    global vx
    global wz
    if start_navigation and not start_mapping:
        vx = msg.linear.x
        wz = msg.angular.z
    elif not start_navigation and not start_mapping:
        vx = 0.0
        wz = 0.0
cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

frame_sim = None
def image_callback(msg):
    global frame_sim
    bridge = CvBridge()
    frame_sim = bridge.imgmsg_to_cv2(msg, "bgr8")
if use_simulator:
    camera_sim_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

# MQTT Set Up
def on_connect(client, userdata, flags, rc):
    client.subscribe(mapping_topic)
    client.subscribe(lidar_topic)
    rospy.loginfo(f"Connected with MQTT Broker at {mqtt_broker_ip}")

def on_message(client, userdata, msg):
    global start_mapping
    global pause_mapping
    global stop_mapping
    global start_navigation
    global compute_slam_process
    global compute_nav_process
    if msg.topic == mapping_topic:
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
    elif msg.topic == lidar_topic:
        data = json.loads(msg.payload.decode("utf-8"))
        enable = bool(data["enable"])
        use_own_map = bool(data["use_own_map"])
        if enable and compute_slam_process is None and compute_nav_process is None:
            if use_own_map:
                rospy.loginfo("Launching compute nav")
                start_navigation = True
                compute_nav_process = subprocess.Popen(["roslaunch", "slam_itbdelabo", "compute_nav.launch", f"use_simulator:={use_simulator}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            else:
                rospy.loginfo("Launching compute slam")
                compute_slam_process = subprocess.Popen(["roslaunch", "slam_itbdelabo", "compute_slam.launch", f"use_simulator:={use_simulator}"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        elif not enable:
            if compute_slam_process is not None:
                os.killpg(os.getpgid(compute_slam_process.pid), signal.SIGTERM)
            if compute_nav_process is not None:
                start_navigation = False
                os.killpg(os.getpgid(compute_nav_process.pid), signal.SIGTERM)
        
mqtt_client = mqtt.Client(client_id=f"slam_node_${random.randint(0,1000)}")
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(mqtt_broker_ip, 1883, 60)

# Utility Function
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

# Main Loop Setup
frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency)
cap = cv2.VideoCapture(camera_id) if not use_simulator else None

# Main Loop
try:
    while not rospy.is_shutdown():
        """convention, rot_vel (+) -> clockwise (navigation/compass-based)"""
        # read camera frame
        if use_simulator:
            ret, frame = frame_sim is not None, frame_sim
        else:
            ret, frame = cap.read()
        if ret:
            compressed_frame = cv2.resize(frame, (camera_width, camera_height))
            _, compressed_frame_encoded = cv2.imencode('.jpg', compressed_frame, [int(cv2.IMWRITE_JPEG_QUALITY), camera_quality])
            compressed_frame_bytes = compressed_frame_encoded.tobytes()
            mqtt_client.publish(camera_topic, compressed_frame_bytes)

        # create msg variables
        hardware_command_msg = HardwareCommand()
        cmd_vel_msg = Twist()
        odom_msg = Odometry()
        
        # inverse kinematics
        vx = constrain(vx, -max_speed_linear, max_speed_linear)
        wz = constrain(wz, -max_speed_angular, max_speed_angular)
        hardware_command_msg.right_motor_speed = (vx*100.0/wheel_radius - wz*wheel_distance/(2.0*wheel_radius))*9.55
        hardware_command_msg.left_motor_speed = (vx*100.0/wheel_radius + wz*wheel_distance/(2.0*wheel_radius))*9.55
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

        # for SLAM simulator mapping
        if  not start_navigation:
            cmd_vel_msg.linear.x = vx
            cmd_vel_msg.angular.z = wz
            cmd_vel_pub.publish(cmd_vel_msg)

        delta_right_angle = right_motor_pulse_delta/gear_ratio*6.28318531
        delta_left_angle = left_motor_pulse_delta/gear_ratio*6.28318531

        # Calculate robot poses based on wheel odometry
        pose_x = pose_x + wheel_radius/2.0 * (delta_right_angle + delta_left_angle) * math.sin(theta);
        pose_y = pose_y + wheel_radius/2.0 * (delta_right_angle + delta_left_angle) * math.cos(theta);
        if not use_imu:
            theta = theta + (delta_right_angle - delta_left_angle) * wheel_radius/wheel_distance;
            theta = warpAngle(theta)
        else:
            theta = heading
        
        #Assign odometry msg
        odom_msg.header.stamp = rospy.Time.now() 
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position = Point(float(pose_x), float(pose_y), 0.0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, theta))
           
        odom_pub.publish(odom_msg)
        hardware_command_pub.publish(hardware_command_msg)
        mqtt_client.loop(timeout=compute_period/1000)
        
        rate.sleep()
except:
    pass
