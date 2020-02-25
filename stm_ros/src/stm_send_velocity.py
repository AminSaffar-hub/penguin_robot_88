#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import serial

linear_vel = 0
angular_vel = 0
def velocity_callback(vel):
    global linear_vel , angular_vel
    linear_vel = vel.linear.x
    angular_vel = vel.angular.z

def convert_to_diff(linear_vel,angular_vel):
    Radius = 0.063
    Length = 0
    V_right = (2*linear_vel + angular_vel * Length) / (2 * Radius)
    V_left = (2*linear_vel - angular_vel * Length) / (2 * Radius)
    return (V_right,V_left)



if __name__=='__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    rospy.init_node('velocity_to_stm',anonymous = True)
    rospy.Subscriber('/cmd_vel',Twist,velocity_callback)
    while(not(rospy.is_shutdown)):
        vel = convert_to_diff(linear_vel,angular_vel)
        vel_msg = vel[0]+" "+vel[1]
        ser.write(str(vel_msg).encode('ascii'))
    
