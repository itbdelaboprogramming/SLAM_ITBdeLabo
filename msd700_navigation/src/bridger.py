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
 


# def compute_pid():
#     sda



def convert_pwm():
    global right_vel, left_vel, right_motor_speed, left_motor_speed
    left_vel    = (vx-wz*wheel_distance/2)/wheel_radius
    right_vel   = (vx+wz*wheel_distance/2)/wheel_radius

    #PWM conversion (this is still brute force using exact value, further implementation better use PID)
    if left_vel>0:
        left_motor_speed = 100
    elif left_vel<0:
        left_motor_speed = -100
    else:
        left_motor_speed = 0
    
    if right_vel>0:
        right_motor_speed = 100
    elif right_vel<0:
        right_motor_speed = -100
    else:
        right_motor_speed = 0

    command_msg.right_motor_speed    = right_motor_speed
    command_msg.left_motor_speed     = left_motor_speed
    command_pub.publish(command_msg)
    rate.sleep()



if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            convert_pwm()
    except rospy.ROSInterruptException:
        rospy.quit()
        pass
