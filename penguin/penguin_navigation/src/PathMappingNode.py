#!/usr/bin/env python 

import rospy
import math
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose 
from std_msgs.msg import String

from penguin_navigation.srv import pathplanner 
from penguin_navigation.srv  import pathplannerRequest 
from penguin_navigation.srv  import pathplannerResponse 

cmd = Twist()
a = String()
i = 1
Po = Pose()
PHI = Po.theta
Kv = 0.5
Kh = 3
def stopp() :

    global cmd,Pub

    cmd_linear = cmd.linear 
    cmd_angular = cmd.angular

    cmd_linear.x = 0.0 
    cmd_linear.y = 0.0 
    cmd_linear.z = 0.0 

    cmd_angular.x = 0.0 
    cmd_angular.y = 0.0 
    cmd_angular.z = 0.0 

    Pub.publish(cmd)

def RunToGoal(Vl,Va) :
    global cmd,Pub
    rate = rospy.Rate(10)
    cmd_linear = cmd.linear 
    cmd_angular = cmd.angular

    cmd_linear.x = Vl
    cmd_linear.y = 0.0 
    cmd_linear.z = 0.0 

    cmd_angular.x = 0.0 
    cmd_angular.y = 0.0 
    cmd_angular.z = Va

    Pub.publish(cmd)
    rate.sleep()
def MoveForward():

    global cmd,Pub

    cmd_linear = cmd.linear 
    cmd_angular = cmd.angular

    cmd_linear.x = 1.0 
    cmd_linear.y = 0.0 
    cmd_linear.z = 0.0 

    cmd_angular.x = 0.0 
    cmd_angular.y = 0.0 
    cmd_angular.z = 0.0 

    Pub.publish(cmd)


def rotate(a) :

    global cmd,Pub
    
    cmd_linear = cmd.linear 
    cmd_angular = cmd.angular

    cmd_linear.x = 0.0 
    cmd_linear.y = 0.0 
    cmd_linear.z = 0.0 

    cmd_angular.x = 0.0 
    cmd_angular.y = 0.0 
    cmd_angular.z = 0.2 

    Pub.publish(cmd)
    
def roll(a) :

    global cmd,Pub
    
    cmd_linear = cmd.linear 
    cmd_angular = cmd.angular

    cmd_linear.x = 0.0 
    cmd_linear.y = 0.0 
    cmd_linear.z = 0.0 

    cmd_angular.x = 0.0 
    cmd_angular.y = 0.0 
    cmd_angular.z = a*0.5 

    Pub.publish(cmd)

def sendresult() :

    global a,PubResult,i

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
    
    PubResult.publish(a)


def GoToGoal2(Xg,Yg,PhiG) :

    global Po,PHI
    OrP = 0
    orientation  = math.atan2(Yg-Po.y,Xg-Po.x)
    erreurPHI = orientation - PHI 
    
    erreurDistX = Xg - Po.x 
    erreurDistY = Yg - Po.y 

    while (not rospy.is_shutdown()) and (math.sqrt(math.pow(erreurDistX,2)+math.pow(erreurDistY,2))>0.01) :
        erreurDistX = Xg - Po.x 
        erreurDistY = Yg - Po.y 
        Vl = Kv * math.sqrt(math.pow(erreurDistX,2)+math.pow(erreurDistY,2)) 
        OrN = math.atan2(erreurDistY,erreurDistX)
        err = math.fabs(OrN -OrP)
        if err > 4 :
            OrN = -OrN
        Va = Kh * (OrN-PHI)
        OrP =OrN
        #print ("erreurDistX = %s erreurDistY = %s"%(erreurDistX,erreurDistY))
        print ("erreurDistX = %s erreurDistY = %s Vl = %f Va = %f OrN = %f"%(erreurDistX,erreurDistY,Vl,Va,OrN))
        RunToGoal(Vl,Va)

    stopp()


def GoToGoal(Xg,Yg,PhiG) :

    global Po

    orientation  = math.atan2(Yg-Po.y,Xg-Po.x)
    erreurPHI = orientation - PHI 

    while (not rospy.is_shutdown()) and ( math.fabs(erreurPHI) > 0.006) :
        erreurPHI = orientation - PHI 
        print ("PHI = %s Erreur = %s"%(PHI,erreurPHI))
        rotate(erreurPHI)
    
    erreurDistX = Xg - Po.x 
    erreurDistY = Yg - Po.y 

    while (not rospy.is_shutdown()) and ((math.fabs(erreurDistX) > 0.01)  or (math.fabs(erreurDistY) > 0.05)) :
        erreurDistX = Xg - Po.x 
        erreurDistY = Yg - Po.y 
        print ("erreurDistX = %s erreurDistY = %s"%(erreurDistX,erreurDistY))
        MoveForward() 

    stopp()

def PoseCallback(P):
    global Po ,PHI
    Po.x = P.x
    Po.y = P.y 
    PHI = P.theta 
    print (PHI)

def handler_server (req):
    global i 
    print("responding....")
    distance  = ( (req.Xr - req.Xg ) * (req.Xr - req.Xg ) )+ ((req.Yr - req.Yg ) *  (req.Yr - req.Yg )) 
    angle = (req.Yg - req.Yr )/ (req.Xg - req.Xr )
    #GoToGoal(req.Xg,req.Yg,0.0)
    GoToGoal2(req.Xg,req.Yg,0.0)
    sendresult()
    i += 1


def initNode():
    global rate , Pub ,PubResult,a
    rospy.init_node('PathMappingNode',anonymous=True)
    Pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    PubResult = rospy.Publisher('/robot',String,queue_size=10)
    s = rospy.Service('PathPlanner',pathplanner,handler_server)
    rospy.Subscriber('/turtle1/pose',Pose,PoseCallback)
    a.data = "zzzzz"
    PubResult.publish(a)
    #rate = rospy.Rate(10)
    #GoToGoal(9.0,9.0,0.0)
    rospy.spin()
    #rate.sleep()


if __name__ == "__main__":
    try :
        initNode()
    except rospy.ROSInternalException :
        pass
    