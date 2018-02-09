#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped

from ImageProcessing import ImageProcessor


class LineFollower:

	def __init__(self):
		#initialize stuff
		self.state_lock = Lock()

		#DEFFO tweak these later
		self.k_p = 3
		self.k_i = 3
		self.k_d = 3
		self.speed = 0.25

		#hook your image processor up to the topic
		self.image_processor = ImageProcessor(self.state_lock)
		self.image_sub = rospy.Subscriber(rospy.get_param("~image_topic", "/camera/color/image_raw"), Image, self.image_processor.image_cb, queue_size=1)

		self.pub_drive = rospy.Publisher(rospy.get_param("~drive_topic", "/vesc/high_level/ackermann_cmd_mux/input/nav_0"), AckermannDriveStamped, queue_size = 1)

	def angle(self):
		self.state_lock.acquire()
		ret = self.k_p*self.image_processor.curr_error + self.k_i*self.image_processor.total_error + self.k_d*self.image_processor.delta_error
		self.state_lock.release()
		return ret

if __name__ == '__main__':
	rospy.init_node("line_follower", anonymous=True)
	lf = LineFollower()

	while not rospy.is_shutdown():
		#drive forward with constant speed and lf.angle
		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"
		msg.drive.steering_angle = lf.angle()
		msg.drive.speed = lf.speed
		print lf.angle()
		lf.pub_drive.publish(msg)