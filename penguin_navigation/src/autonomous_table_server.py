#!/usr/bin/env python
import rospy 
import actionlib 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import * 
from geometry_msgs.msg import Point
from std_msgs.msg import String
from penguin_navigation.msg import tables
import tf

a = String()
s = String()
j=1
i=0
t1=""
t2=""
t3=""
t4=""
t5=""

def tables_string(req):
    global t1,t2,t3,t4,t5,i
    if (j==1):
        t1=req.table1
        t2=req.table2
        t3=req.table3
        t4=req.table4
        t5=req.table5
        rospy.loginfo("  tab1= %s , tab2= %s ,tab3= %s , tab4= %s, tab5 = %s " 
        % (req.table1, req.table2,req.table3,req.table4,req.table5))
        s.data= req.table1 + req.table2+req.table3+req.table4+req.table5
        print(s.data)
        path()
        j=0

def sendresult() :
    global a,i
    if i == 1 : 
        a.data = "azzzz"
    elif i == 2 : 
        a.data = "aazzz"
    elif i == 3 : 
        a.data = "aaazz"
    elif i == 4 : 
        a.data = "aaaaz"
    elif i == 5 : 
        a.data = "aaaaa"   
    PubResult = rospy.Publisher('/robot',String,queue_size=10)
    PubResult.publish(a)

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
        sendresult()
        return True
    else:
        rospy.loginfo("goal_failed")
        return False


def path():
    global t1,t2,t3,t4,t5,i,j
    if(t1=='1'):
        i=1
        move_to_goal(3.0 , 3.0 , 1.57)
    if(t1=='2'):
        i=2
        move_to_goal(1.0 , 1.0 , 0)
    if(t1=='3'):
        i=3
        move_to_goal(1.0 , 1.0 , 1.57)
    if(t1=='4'):
        i=4
        move_to_goal(1.0 , 1.0 , 1.57)
    if(t1=='5'):
        i=5
        move_to_goal(1.0 , 1.0 , 1.57)
    j=1
        

    
def node():

    rospy.init_node('AutonomousServer', anonymous=True)
    PubResult = rospy.Publisher('/robot',String,queue_size=10)
    a.data = "zzzzz"
    PubResult.publish(a)
    #move_to_goal(3.0 , 3.0 , 1.57)
    rospy.Subscriber("/tables_to_serve", tables, tables_string)
    rospy.spin()
    


if __name__ =='__main__' :
    try:   
        node()
    except rospy.ROSInternalException :
        pass