#!/usr/bin/env python3

import rospy
from slam_itbdelabo.msg import HardwareCommand
from geometry_msgs.msg import Twist
from slam_itbdelabo.srv import SetMapping, SetMappingResponse, SetMappingRequest

# Global Variables
start_mapping = False
stop_mapping = False
pause_mapping = False

# Initialize ROS Node
rospy.init_node('slam_node')

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

# Create ROS Service
def handle_mapping(req: SetMappingRequest):
    global start_mapping
    global pause_mapping
    global stop_mapping
    if sum([req.start, req.pause, req.stop]) > 1:
        success = False
        response = SetMappingResponse(success)
        return response
    else:
        start_mapping = False
        pause_mapping = False
        stop_mapping = False
        if req.start:
            start_mapping = True
        elif req.pause:
            pause_mapping = True
        elif req.stop:
            stop_mapping = True
        success = True
        response = SetMappingResponse(success)
        return response
rospy.Service('set_mapping', SetMapping, handle_mapping)

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

# Get ROS Parameters (loaded from slam.yaml)
compute_period = rospy.get_param("/compute_period")
max_speed_linear = rospy.get_param("/max_speed_linear")
max_speed_angular = rospy.get_param("/max_speed_angular")
wheel_radius = rospy.get_param("/wheel_radius","2.75")		# in cm
wheel_distance = rospy.get_param("/wheel_distance","23.0")		# in cm

frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency)

while not rospy.is_shutdown():
    # TODO: add logic for start mapping, pause mapping, and stop mapping
    print(f"start_mapping = {start_mapping}, pause_mapping = {pause_mapping}, stop_mapping = {stop_mapping}")
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
