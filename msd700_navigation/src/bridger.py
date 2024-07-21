#!/usr/bin/env python3

"""
This code converts cmd_vel to hardware command
1. Subscribe to cmd_vel topic
2. Compute angular velocity for each motor
3. Publish to HardwareCommand


Additional
1. include yaml file to execute params
2. use PID, put PID parameters in yaml
"""


import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from ros_msd700_msgs.msg import HardwareCommand
import numpy as np



# Initialize ROS Node
rospy.init_node('bridger')

# Get parameters
wheel_distance = rospy.get_param("/raw_sensor/wheel_distance", 23.0)    # in cm
wheel_radius = rospy.get_param("/raw_sensor/wheel_radius", 2.75)	  # in cm

# Initialization
vx = 0.0
vy = 0.0
vz = 0.0
wx = 0.0
wy = 0.0
wz = 0.0
left_vel = 0.0
right_vel = 0.0



# Main loop Setup
rate = rospy.Rate(100)

# Publisher
command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
command_msg = HardwareCommand()

# Subscriber
def cmd_vel_callback(msg: Twist):
    global vx, vy, vz, wx, wy, wz
    vx = msg.linear.x
    vy = msg.linear.y
    vz = msg.linear.z
    wx = msg.angular.x
    wy = msg.angular.y
    wz = msg.angular.z
cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

def actual_vel_callback(msg:Twist):
    global vx_act, vy_act, vz_act, wx_Act, wy_Act, wz_Act
    vx_act = msg.linear.x
    vy_act = msg.linear.y
    vz_act = msg.linear.z
    wx_Act = msg.angular.x
    wy_Act = msg.angular.y
    wz_Act = msg.angular.z
actual_vel_sub = rospy.Subscriber("/odom", Twist, actual_vel_callback)
 

def map_value(x, in_min, in_max, out_min, out_max):
    return np.interp(x, [in_min, in_max], [out_min, out_max])


# in_min  = -12
# in_max  = 12.0
in_min  = -0.002
in_max  = 0.002
out_min = -150 
out_max = 150
mul_factor = 10 # add more speed
rotation_factor = 10

def convert_pwm():
    global right_vel, left_vel, right_motor_speed, left_motor_speed
    left_vel    = (vx-wz*wheel_distance/(2*100)*rotation_factor)/(wheel_radius/100) * mul_factor    # converted to meter
    right_vel   = (vx+wz*wheel_distance/(2*100)*rotation_factor)/(wheel_radius/100) * mul_factor

    #PWM conversion (this is still brute force using exact value, further implementation better use PID)
    # left_vel    = (vx-wz*wheel_distance/(2*10))/(wheel_radius*100)     # converted to meter
    # right_vel   = (vx+wz*wheel_distance/(2*10))/(wheel_radius*100)
    # left_motor_speed    = map_value(left_vel, in_min, in_max, out_min, out_max)
    # right_motor_speed   = map_value(right_vel, in_min, in_max, out_min, out_max)

    # left_motor_speed    = left_vel
    # right_motor_speed   = right_vel
    
    # if left_vel>0:
    #     left_motor_speed = 100
    # elif left_vel<0:
    #     left_motor_speed = -100
    # else:
    #     left_motor_speed = 0
    
    # if right_vel>0:
    #     right_motor_speed = 100
    # elif right_vel<0:
    #     right_motor_speed = -100
    # else:
    #     right_motor_speed = 0

    command_msg.right_motor_speed   = right_vel # right_motor_speed #  
    command_msg.left_motor_speed    = left_vel # left_motor_speed  #   
    command_pub.publish(command_msg)
    rate.sleep()

# def convert_pwm_PID():
#     delta_right_angle   = (2*np.pi * wheel_radius / 100 ) * (right_motor_pulse_delta / gear_ratio)
#     delta_left_angle    = (2*np.pi * wheel_radius / 100 ) * (left_motor_pulse_delta / gear_ratio)



if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            convert_pwm()
    except rospy.ROSInterruptException:
        rospy.quit()
        pass
