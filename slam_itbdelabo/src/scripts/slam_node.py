#!/usr/bin/env python3

import rospy
from slam_itbdelabo.msg import HardwareCommand
from geometry_msgs.msg import Twist
from slam_itbdelabo.srv import Mapping, MappingResponse, MappingRequest

# Initialize ROS Node
rospy.init_node('slam_node')

# Create ROS Publisher
hardware_command_pub = hardware_command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)

# Create ROS Subscriber
x_vel = 0.0
rot_vel = 0.0
def cmd_vel_callback(msg):
    global x_vel
    global rot_vel
    x_vel = msg.linear.x
    rot_vel = msg.angular.z
cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

# Create ROS Service
start_mapping = False
stop_mapping = False
pause_mapping = False
def handle_mapping(req: MappingRequest):
    global start_mapping
    global pause_mapping
    global stop_mapping
    if sum([req.start, req.pause, req.stop]) > 1:
        success = False
        response = MappingResponse(success)
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
        response = MappingResponse(success)
        return response
rospy.Service('mapping', Mapping, handle_mapping)

# Get ROS Parameters (loaded from slam.yaml)
compute_period = rospy.get_param("/compute_period")
max_speed_linear = rospy.get_param("/max_speed_linear")
max_speed_angular = rospy.get_param("/max_speed_angular")

frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency)

while not rospy.is_shutdown():
    # convention, rot_vel (+) -> clockwise (navigation/compass-based)
    hardware_command_msg = HardwareCommand()
    if x_vel > 0 :
        # forward
        hardware_command_msg.movement_command = 3 
    elif rot_vel < 0:
        # left
        hardware_command_msg.movement_command = 2 
    elif rot_vel > 0:
        # right
        hardware_command_msg.movement_command = 1 
    else:
        # stop
        hardware_command_msg.movement_command = 0
    
    hardware_command_pub.publish(hardware_command_msg)
    
    rate.sleep()