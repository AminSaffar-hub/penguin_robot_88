#!/usr/bin/env python
import rospy 
import actionlib 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import * 
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int8
from penguin_navigation.msg import tables
import tf
import os

i= 0
point_list = []

def beds_path_callback(point):
    global point_list
    print("tkalettttttt")
    point_list.append(point)
    rospy.loginfo("new bed point received !")
    rospy.loginfo(str(point_list))


def move_to_goal(xGoal , yGoal , orientation):
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0)) ):
        rospy.loginfo("still waiting")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    quaternion = tf.transformations.quaternion_from_euler(0,0,orientation)
    goal.target_pose.pose.position = Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    rospy.loginfo("sending goal location")
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))
    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("goal_succeded")
        return True
    else:
        rospy.loginfo("goal_failed")
        return False



    
def node():
    global i,point_list
    rospy.init_node('AutonomousMove', anonymous=True)
    state_pub = rospy.Publisher('/state', Int8, queue_size=10)
    i= 0
    state_pub.publish(i)
    rospy.Subscriber('/bed_point', Point, beds_path_callback,queue_size=10)
    #rospy.sleep(6)
    
    #room1
    move_to_goal(5.0 , -4.0 , 1.57)
    i= 1
    state_pub.publish(i)
    rospy.sleep(2)
    move_to_goal(5.0 , -4.0 , 0)
    move_to_goal(5.0 , -4.0 , -3)

    #shut down detection
    i= 0
    state_pub.publish(i)
    rospy.sleep(3)
    
    print(len(point_list))
    for point in point_list :
        move_to_goal(point.x ,point.y, 0.0 )
    point_list = []

    os.system("rosnode kill /move_base" )
    os.system("rosrun drsina_navigation autonomous_navigation_with_detection2.py")
    #rospy.sleep(10)

    """ move_to_goal(7.67 , -4.15 , 1.57)
    rospy.sleep(2)
    move_to_goal(6.88 , -7.2 , 0)
    rospy.sleep(2)
    move_to_goal(4.85 , -7.51 , -3.1)
    rospy.sleep(2) """
    
    #room2
    """move_to_goal(-1.0 , -4.0 , 1.57)
    move_to_goal(2.7 , 3.44 , 1.57)

    i= 1
    state_pub.publish(i)

    move_to_goal(2.7 , 3.44 , 3.14)
    move_to_goal(2.7 , 3.44 , 0)

    i= 0
    state_pub.publish(i)
    rospy.sleep(2)
    print(len(point_list))
    for point in point_list :
        move_to_goal(point.x ,point.y, 0.0 )
    point_list = []"""
    """move_to_goal(2.32 , 7.16 , 1.57)
    rospy.sleep(2)
    move_to_goal(6.98 , 8.54 , 0)
    rospy.sleep(2)"""


    """
    #room3
    move_to_goal(-4.33 , 3.27 ,1.57)

    i= 1
    state_pub.publish(i)

    move_to_goal(-4.33 , 3.27 ,3.14)
    move_to_goal(-4.33 , 3.27 ,0)

    i= 0
    state_pub.publish(i)

    move_to_goal(-8.21 , 7.22 , 1.57)
    rospy.sleep(2)
    move_to_goal(-8.21 , 7.22 , -1.57)
    move_to_goal(-1.47 , 8.75 , 3.14)
    rospy.sleep(2)"""

    #rospy.spin()

    


if __name__ =='__main__' :
    try:   
        node()
    except rospy.ROSInternalException :
        pass