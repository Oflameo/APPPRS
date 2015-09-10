#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped


def callback(data):
    s = (str(data.pose.position.x)+', '+
         str(data.pose.position.y)+', '+
         str(data.pose.position.z)+', '+
         str(data.pose.orientation.x)+', '+
         str(data.pose.orientation.y)+', '+
         str(data.pose.orientation.z)+', '+
         str(data.pose.orientation.w))
    f.write(s+'\n')
    print(s)

rospy.init_node('listener', anonymous=True)
f = open('/tmp/waypoints.txt', 'w')
rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
rospy.spin()
f.close()