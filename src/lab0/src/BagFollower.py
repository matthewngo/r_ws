#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

BAG_TOPIC = 'vesc/low_level/ackermann_cmd_mux/input/teleop'# Name of the topic that should be extracted from the bag
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PUB_RATE = 60# The rate at which messages should be published

def follow_bag(bag_path, follow_backwards=False):
	#pass
        bag = rosbag.Bag(bag_path)
        pub = rospy.Publisher(BAG_TOPIC, AckermannDriveStamped, queue_size=10)
        r = rospy.Rate(PUB_RATE)
        for topic, msg, t in bag.read_messages():
            pub.publish(msg)
            print msg
            r.sleep()

if __name__ == '__main__':
	bag_path = rospy.get_param("bag_file") # The file path to the bag file
	follow_backwards = False # Whether or not the path should be followed backwards
	
	rospy.init_node('bag_follower', anonymous=True)
	
	# Populate param(s) with value(s) passed by launch file
	
	follow_bag(bag_path, follow_backwards)
