#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
from std_msgs.msg import String

msg="STOP"


def chatter_callback(message):
    global msg
    rospy.loginfo(" The button pressed is:  %s", message.data)
    msg= message.data



def manualMode (msg):
    velocity_msg = Twist()
    if msg=="TOP": 
        velocity_msg.linear.x=5.0
    else:
        if msg == "DOWN":
            velocity_msg.linear.x=-5.0
        else:
            if msg == "RIGHT":
                velocity_msg.angular.z=5.0
            else:
                if msg == "LEFT":
                    velocity_msg.angular.z=-5.0
                else:
                    velocity_msg.angular.z=0.0
                    velocity_msg.linear.x=0.0

    velocity_publisher.publish (velocity_msg)

    



if __name__ == "__main__":
    try:
        rospy.init_node('ManualMode',anonymous=True)
        velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist,queue_size=10)
        rospy.Subscriber("/Command", String, chatter_callback)
        manualMode (msg)
        rospy.spin()
        
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated") 