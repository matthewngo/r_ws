#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock

from sensor_msgs.msg import Image

from ImageProcessing import ImageProcessor


class LineFollower:

	def __init__(self):
		#initialize stuff
		self.state_lock = Lock()

		#DEFFO tweak these later
		self.k_p = 3
		self.k_i = 3
		self.k_d = 3

		#hook your image processor up to the topic
		self.image_processor = ImageProcessor(self.state_lock)
		self.image_sub = rospy.Subscriber(rospy.get_param("~image_topic", "/camera/color/image_raw"), Image, self.image_processor.image_cb, queue_size=1)

	def angle(self):
		#return the angle you go to
	
if __name__ == '__main__':
	rospy.init_node("line_follower", anonymous=True)
	lf = LineFollower()

	while not rospy.is_shutdown():
		#drive forward with constant speed and lf.angle