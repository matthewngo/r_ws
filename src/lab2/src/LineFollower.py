#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

from ImageProcessing import ImageProcessor, TemplateMatcher
from ImageProcessingExtra import ImageProcessorExtra

EXTRA_CRED_GRAYSCALE = True 

class LineFollower:

	def __init__(self):
		#initialize stuff
		self.state_lock = Lock()

		"""
		self.k_p = 1.25
		self.k_i = 0
		self.k_d = 0.4
		"""

		self.k_p = 1
		self.k_i = 0
		self.k_d = 0.5
		self.speed = 0.5

		self.lineFollow = True

		#hook your image processor up to the topic
		if self.lineFollow:
			if EXTRA_CRED_GRAYSCALE:
				self.image_processor = ImageProcessorExtra(self.state_lock)
				print "get grayed"
				self.image_sub = rospy.Subscriber("/camera/fisheye/image_raw", Image, self.image_processor.image_cb, queue_size=1)
			else:
				print "line_follow_color"
				self.image_processor = ImageProcessor(self.state_lock)
				self.image_sub = rospy.Subscriber(rospy.get_param("~image_topic", "/camera/color/image_raw"), Image, self.image_processor.image_cb, queue_size=1)
		else:
			self.template_matcher = TemplateMatcher(self.state_lock)
			self.template_sub = rospy.Subscriber(rospy.get_param("~image_topic", "/camera/color/image_raw"), Image, self.template_matcher.image_cb, queue_size=1)

		self.pub_drive = rospy.Publisher(rospy.get_param("~drive_topic", "/vesc/high_level/ackermann_cmd_mux/input/nav_0"), AckermannDriveStamped, queue_size = 1)

	def angle(self):
		self.state_lock.acquire()
		if self.image_processor.curr_error <= -0.999:
			ret = 0
		else :
			ret = self.k_p*self.image_processor.curr_error + self.k_i*self.image_processor.total_error + self.k_d*self.image_processor.delta_error
		self.state_lock.release()
		return -1 * ret

if __name__ == '__main__':
	rospy.init_node("line_follower", anonymous=True)
	lf = LineFollower()

	while not rospy.is_shutdown():
		#drive forward with constant speed and lf.angle
		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"

		if lf.lineFollow:
			if lf.image_processor.visible == False:
				msg.drive.steering_angle = 0
				msg.drive.speed = -1 * lf.speed
			else:
				msg.drive.steering_angle = lf.angle()
				msg.drive.speed = lf.speed
		else:
			ang = lf.template_matcher.choose_template()
			if lf.template_matcher.visible == False:
				msg.drive.steering_angle = 0
				msg.drive.speed =  lf.speed #-1*lf.speed
			else:
				msg.drive.steering_angle = ang
				msg.drive.speed = lf.speed
			#rospy.sleep(0.5)
			
		#print lf.angle()
		lf.pub_drive.publish(msg)
