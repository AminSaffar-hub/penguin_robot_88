#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def chatter_callback(message):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", message.data)
    
def remote_control_reciever():

    rospy.init_node('remote_control_reciever', anonymous=True)

    rospy.Subscriber("/chatter", String, chatter_callback)

    rospy.spin()

if __name__ == '__main__':
<<<<<<< HEAD
    remote_control_reciever()
=======
    remote_control_reciever()
>>>>>>> abdessalem
