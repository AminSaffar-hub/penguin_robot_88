#!/usr/bin/env python
import rospy
from penguin_navigation.msg import XyTheta

def callback(data):
    rospy.loginfo("  x= %f , y= %f , theta = %f " % (data.x, data.y,data.theta))
     
def listener():
    rospy.init_node('AutonomousReceiver', anonymous=True)
    rospy.Subscriber("/odomCommand", XyTheta, callback)
  
    rospy.spin()

if __name__ == '__main__':
    listener()