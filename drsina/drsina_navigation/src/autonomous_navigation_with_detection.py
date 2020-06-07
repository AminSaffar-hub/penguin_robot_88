#!/usr/bin/env python
import rospy 
import actionlib 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import * 
from geometry_msgs.msg import Point
from std_msgs.msg import String
from penguin_navigation.msg import tables
import tf

point_list = []

def beds_path_callback(point):
    global point_list
    point_list.append(point)
    print("new bed point received !")
    print(point_list)


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
    global a,subscrib
    rospy.init_node('AutonomousMove', anonymous=True)
    subscriber = rospy.Subscriber("/bed_point", Point, beds_path_callback)
    #move_to_goal(2.2 , -2.5 , 0)
    #move_to_goal(6.7 , 3.4 , 1.57)
    #move_to_goal(-5.4 , 3.4 , 1.57)

    rospy.spin()

    


if __name__ =='__main__' :
    try:   
        node()
    except rospy.ROSInternalException :
        pass