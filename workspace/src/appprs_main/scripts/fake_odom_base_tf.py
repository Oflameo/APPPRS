#!/usr/bin/env python  
import roslib
import rospy
import tf

rospy.init_node('tf_broadcaster_odom_base_link')
r = rospy.Rate(100) # 100hz
br = tf.TransformBroadcaster()
first_time = 0;
vel = 0.1
while not rospy.is_shutdown():
    if not first_time:
	first_time = rospy.Time.now()
    x = (rospy.Time.now()-first_time).to_sec()*0.1
    br.sendTransform((x, 0.0, 0.5),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
    print("current x: "+str(x))
    r.sleep()

