#!/usr/bin/python

import rosbag
import sys

bag = rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages(topics=['scan']):
    print(msg)
bag.close()
