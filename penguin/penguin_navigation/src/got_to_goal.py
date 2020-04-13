#!/usr/bin/env python


import rospy 
import actionlib 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import * 
from geometry_msgs.msg import Point
import tf

from penguin_navigation.srv import  goal_srv
from penguin_navigation.srv import  goal_srvRequest
from penguin_navigation.srv import  goal_srvResponse

new_goal = Point()
n_goal = False

def move_to_goal(xGoal , yGoal , orientation):
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)

    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0)) ):
        rospy.loginfo("still waiting")
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"
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
def goal_responce(req):
    global new_goal,n_goal
    new_goal.x = req.x
    new_goal.y = req.y
    new_goal.z = req.theta
    n_goal = True
    return goal_srvResponse()
def node():
    global n_goal
    rospy.init_node('map_navigation',anonymous=False) 
    rospy.Service('autonomous_navigation', goal_srv, goal_responce) 
    while not(rospy.is_shutdown()):
        if( n_goal ):
            n_goal = False
            move_to_goal(new_goal.x,new_goal.y,new_goal.z)


if __name__ =='__main__' :
    node()
    #x_goal = 2
    #y_goal = 2
    #move_to_goal(x_goal,y_goal,0)
    #x_goal = 0
    #y_goal = 2
    #move_to_goal(x_goal,y_goal,0)
    #x_goal = 0
    #y_goal = 0
    #move_to_goal(x_goal,y_goal,3.14)
    #rospy.spin()





