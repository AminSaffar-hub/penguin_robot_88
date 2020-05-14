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
a.data = "zzzzz"
z=""

table1x = 1.76 
table1y = 1.29

table2x = -0.6 
table2y = -1.73

table3x = 2.6
table3y = -3.0

table4x = -0.7
table4y = -5.4

table5x = 1.27
table5y = -6.187

table6x = 0.0
table6y = -8.9

table7x = -1.9
table7y = -9.3

table8x = 1.95
table8y = -8.706


button ="a"
def tables_string(req):
    global t1,t2,t3,t4,t5,i,j
    button = req.b
    print(button)
    if (j==1) and (button=="g"):
        t1=req.table1
        t2=req.table2
        t3=req.table3
        t4=req.table4
        t5=req.table5
        rospy.loginfo("  tab1= %s , tab2= %s ,tab3= %s , tab4= %s, tab5 = %s " 
        % (req.table1, req.table2,req.table3,req.table4,req.table5))
        s.data= req.table1 + req.table2+req.table3+req.table4+req.table5
        print(s.data)
        #subscrib.unregister()
        j = 0
        path()


def sendresult() :
    global a,i1,t2,t3,t4,t5,z
    if i == 1 : 
        a.data = "azzzzz"
    elif i == 2 : 
        a.data = a.data[:1] + "a" + a.data[2:]
    elif i == 3 : 
        a.data = a.data[:2] + "a" + a.data[3:]
    elif i == 4 :  
        a.data = a.data[:3] + "a" + a.data[4:]
    elif i == 5 :
        a.data = a.data[:4] + "a" + a.data[5:]  
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
    global t1,t2,t3,t4,t5,i,j,subscrib,z
    if(t1=='1'):
        z = z+"a"
        i=1
        move_to_goal(table1x , table1y , 1.57)
    if(t2=='2'):
        z=z+"a"
        i=2
        move_to_goal(table2x, table2y , 0)
    if(t3=='3'):
        z=z+"a"
        i=3
        move_to_goal(table3x , table3y , 1.57)
    if(t4=='4'):
        z=z+"a"
        i=4
        move_to_goal(table4x , table4y , 1.57)
    if(t5=='5'):
        z=z+"a"
        i=5
        move_to_goal(table5x , table5y , 1.57)
    j=1
    z=""
    #subscrib = rospy.Subscriber("/tables_to_serve", tables, tables_string,queue_size = 5)
        

    
def node():
    global a,subscrib
    rospy.init_node('AutonomousServer', anonymous=True)
    PubResult = rospy.Publisher('/robot',String,queue_size=10)
    a.data = "zzzzz"
    PubResult.publish(a)
    #move_to_goal(3.0 , 3.0 , 1.57)
    subscrib = rospy.Subscriber("/tables_to_serve", tables, tables_string,queue_size = 5)
    move_to_goal(table1x , table1y , 1.57)
    move_to_goal(table2x, table2y , 0)
    move_to_goal(table3x , table3y , 1.57)
    move_to_goal(table4x , table4y , 1.57)
    move_to_goal(table5x , table5y , 1.57)
    move_to_goal(table6x , table6y , 2)
    move_to_goal(table7x , table7y , 2)
    move_to_goal(table8x , table8y , -0.75)

    rospy.spin()

    


if __name__ =='__main__' :
    try:   
        node()
    except rospy.ROSInternalException :
        pass