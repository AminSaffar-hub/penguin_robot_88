#!/usr/bin/env python

from penguin_navigation.srv import tables_path
from penguin_navigation.srv import tables_pathRequest
from penguin_navigation.srv import tables_pathResponse

import rospy

def handle_tables_path(req):
    print "Returning [%s + %s + %s + %s +%s = %s]"%(req.table1, req.table2,req.table3, req.table4,req.table5
     (req.table1 + req.table2 + req.table3 + req.table4 + req.table5))
    return AddTwoIntsResponse(req.table1 + req.table2 + req.table3 + req.table4 + req.table5)

def tables_server():
    rospy.init_node('tableees_server')
    s = rospy.Service('/tables_to_serve', tables_path, handle_tables_path)
    print "Ready"
    rospy.spin()
    
if __name__ == "__main__":
    tables_server()