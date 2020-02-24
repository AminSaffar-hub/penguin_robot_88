#!/usr/bin/env python


import serial
import rospy



def  serial_node():
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    rospy.init_node('test_sender_node',anonymous = True)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        ser.write(b"abc")    
        rate.sleep()


if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
