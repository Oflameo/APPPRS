#!/usr/bin/python

import rosbag
import sys

bag = rosbag.Bag(sys.argv[1])
out = rosbag.Bag(sys.argv[1]+".new", 'w')
for topic, msg, t in bag.read_messages(topics=['/scan']):
    #print(msg)
    out.write('scan', msg, t)
bag.close()
