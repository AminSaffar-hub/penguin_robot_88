#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0.0
y=0.0
theta=0.0

def poseCallback(Odom_msg):
    global x,y,theta
    x= Odom_msg.pose.pose.position.x
    y= Odom_msg.pose.pose.position.y
    theta= Odom_msg.pose.pose.orientation.z
    print("x=" ,x)
    print("y=" ,y)
    print("theta=" ,theta)

def move (speed,distance,is_forward):
    global x,y
    velocity_msg = Twist()
    x0=x
    y0=y

    if(is_forward):
        velocity_msg.linear.x= abs(speed)
    else:
        velocity_msg.linear.x= -abs(speed)
    
    
    distance_moved = 0.0
    loop_rate =rospy.Rate(10)
    t0=rospy.Time.now().to_sec()
    velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist,queue_size=10)
    while True:
        velocity_publisher.publish (velocity_msg)
        rospy.loginfo("Husky moves forward")
        loop_rate.sleep()
        t1=rospy.Time.now().to_sec()
        distance_moved = (t1-t0)*speed  
        print distance_moved
        if not(distance_moved < distance):
            rospy.loginfo("reached")
            break
    
    velocity_msg.linear.x=0
    velocity_publisher.publish(velocity_msg)

def rotate (angular_speed_degree, relative_angle_degree,bool):
    global theta
    velocity_msg =Twist()
    velocity_msg.linear.x=0
    velocity_msg.linear.y=0
    velocity_msg.linear.z=0
    velocity_msg.angular.x=0
    velocity_msg.angular.y=0
    velocity_msg.angular.z=0
    thetha0=theta
    angular_speed = math.radians(abs(angular_speed_degree))
    if (bool):
        velocity_msg.angular.z= abs(angular_speed)
    else:
        if (bool==0):
            velocity_msg.angular.z= -abs(angular_speed)
    angle_moved = 0.0
    loop_rate =rospy.Rate(10)
    velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist,queue_size=10)
    t0=rospy.Time.now().to_sec()
    while True:
        velocity_publisher.publish (velocity_msg)
        rospy.loginfo("Husky rotate")
        loop_rate.sleep()
        t1=rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree 
        print current_angle_degree
        if (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break
    
    velocity_msg.angular.z=0
    velocity_publisher.publish(velocity_msg)


def go_to_goal(xgoal,ygoal):
    global x,y,theta
    velocity_msg=Twist()
    loop_rate =rospy.Rate(10)
    while(True):
        K_linear=1.0
        distance=abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        linear_speed = distance * K_linear
        K_angular=4.0
        desired_angle_goal=math.atan2(ygoal-y,xgoal-x)
        angular_speed=(desired_angle_goal-theta)*K_angular
        velocity_msg.linear.x = linear_speed
        velocity_msg.angular.z = angular_speed
        velocity_publisher.publish(velocity_msg)
        print('x=',x,'y=',y)
        if(x>(xgoal-0.2) and x<(xgoal+0.2) and y>(ygoal-0.2) and y<(ygoal+0.2)):
            rospy.loginfo("reached")
            break
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    velocity_publisher.publish(velocity_msg)

if __name__ == "__main__":
    try:
        rospy.init_node('husky_motion_pose',anonymous=True)
        velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist,queue_size=10)
        pose_subsriber = rospy.Subscriber("/husky_velocity_controller/odom",Odometry,poseCallback)
        time.sleep(2)
        move(4.0,1.0,True)
        time.sleep(5)
        rotate(10,90,0)
        #go_to_goal(4.0,4.0)

    except rospy.ROSInterruptException: 
        rospy.loginfo("node terminated") 