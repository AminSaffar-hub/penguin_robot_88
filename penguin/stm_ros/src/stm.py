#!/usr/bin/env python

import serial 
import rospy
from std_msgs.msg import String




def  serial_node():
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    pub = rospy.Publisher('/test',String,queue_size=10)
    rospy.init_node('test_node',anonymous = True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        raw_str = ser.readline()
        odom_str = raw_str.split()
        rospy.loginfo(' x= '+odom_str[0]+' y= '+odom_str[1]+' theta= '+odom_str[2])
        pub.publish(' x= '+odom_str[0]+' y= '+odom_str[1]+' theta= '+odom_str[2])
        rate.sleep()


if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass

