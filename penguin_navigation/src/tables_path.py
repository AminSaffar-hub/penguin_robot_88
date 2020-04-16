#!/usr/bin/env python


import rospy
from penguin_navigation.msg import tables
from std_msgs.msg import String

s = String()



def callback(req):
    rospy.loginfo("  tab1= %s , tab2= %s ,tab3= %s , tab4= %s, tab5 = %s " % (req.table1, req.table2,req.table3,req.table4,req.table5))
    s.data= req.table1 + req.table2+req.table3+req.table4+req.table5
    print(s.data)
    

    
def node():

    rospy.init_node('tablessubscriber', anonymous=True)
    rospy.Subscriber("/tables_to_serve", tables, callback)
    rospy.spin()


if __name__ =='__main__' :
    node()

