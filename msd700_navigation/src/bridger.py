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

def convert_pwm():
    global right_vel, left_vel, right_motor_speed, left_motor_speed
    # left_vel    = (vx-wz*wheel_distance/(2*100))/(wheel_radius/100)     # converted to meter
    # right_vel   = (vx+wz*wheel_distance/(2*100))/(wheel_radius/100)

    #PWM conversion (this is still brute force using exact value, further implementation better use PID)
    left_vel    = (vx-wz*wheel_distance/(2*10))/(wheel_radius*100)     # converted to meter
    right_vel   = (vx+wz*wheel_distance/(2*10))/(wheel_radius*100)
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


# class PID:
#     def __init__(self, kp, ki, kd):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.previous_error = 0
#         self.integral = 0

#     def compute(self, setpoint, measured_value, dt):
#         error = setpoint - measured_value
#         self.integral += error * dt
#         derivative = (error - self.previous_error) / dt
#         output = self.kp * error + self.ki * self.integral + self.kd * derivative
#         self.previous_error = error
#         return output

# class MotorController:
#     def __init__(self):
#         rospy.init_node('motor_controller', anonymous=True)

#         # PID parameters
#         self.left_pid = PID(kp=1.0, ki=0.0, kd=0.0)
#         self.right_pid = PID(kp=1.0, ki=0.0, kd=0.0)

#         # Subscriptions
#         self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
#         self.actual_speed_sub = rospy.Subscriber('/actual_speed', Twist, self.actual_speed_callback)

#         # Publisher
#         self.hardware_cmd_pub = rospy.Publisher('/HardwareCommand', HardwareCommand, queue_size=10)

#         # Variables
#         self.cmd_vel = Twist()
#         self.actual_speed = Twist()

#     def cmd_vel_callback(self, msg):
#         self.cmd_vel = msg

#     def actual_speed_callback(self, msg):
#         self.actual_speed = msg

#     def compute_pwm(self):
#         # Differential drive calculations
#         wheel_base = 0.5  # Distance between wheels
#         wheel_radius = 0.1  # Radius of the wheels

#         linear_velocity = self.cmd_vel.linear.x
#         angular_velocity = self.cmd_vel.angular.z

#         left_speed_setpoint = (linear_velocity - angular_velocity * wheel_base / 2) / wheel_radius
#         right_speed_setpoint = (linear_velocity + angular_velocity * wheel_base / 2) / wheel_radius

#         dt = 1.0 / 10.0  # Assuming a control loop of 10 Hz

#         left_pwm = self.left_pid.compute(left_speed_setpoint, self.actual_speed.linear.x, dt)
#         right_pwm = self.right_pid.compute(right_speed_setpoint, self.actual_speed.linear.x, dt)

#         # Create HardwareCommand message
#         hardware_cmd = HardwareCommand()
#         hardware_cmd.left_pwm = left_pwm
#         hardware_cmd.right_pwm = right_pwm

#         return hardware_cmd

#     def run(self):
#         rate = rospy.Rate(10)  # 10 Hz
#         while not rospy.is_shutdown():
#             hardware_cmd = self.compute_pwm()
#             self.hardware_cmd_pub.publish(hardware_cmd)
#             rate.sleep()



if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            convert_pwm()
    except rospy.ROSInterruptException:
        rospy.quit()
        pass
