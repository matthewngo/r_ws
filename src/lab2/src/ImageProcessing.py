#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock

from sensor_msgs.msg import Image

import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError



class ImageProcessor:

	def __init__(self, state_lock=None):
		#initialize stuff
		if state_lock is None:
			self.state_lock = Lock()
		else:
			self.state_lock = state_lock

		self.bridge = CvBridge()

		#error goes from -1 (all the way to the left side of the image) to +1 (all the way to the right side of the image)
		self.curr_error = 0
		self.total_error = 0
		self.delta_error = 0
		self.prev_error = 0

		self.prev_msg = None

		self.pub_masked = rospy.Publisher("/lf/viz/masked", Image, queue_size = 1)

	def image_cb(self, msg):
		self.state_lock.acquire()

		if(self.prev_msg == None):
			self.prev_msg = msg
			self.state_lock.release()
			return

		#image processing:
		#convert message to cv2
		"""print "------"
		print msg.header
		print msg.height
		print msg.width
		print msg.encoding
		print msg.is_bigendian
		print msg.step
		print len(msg.data)"""

		im = self.bridge.imgmsg_to_cv2(msg)

		#convert rgb to hsv
		im_hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
		
		#threshold red/blue hues; everything else is black
		red_1 = cv2.inRange(im_hsv, np.array([0,100,100]), np.array([10,255,255]))
		red_2 = cv2.inRange(im_hsv, np.array([169,100,100]), np.array([179,255,255]))
		blue = cv2.inRange(im_hsv, np.array([90,100,100]), np.array([120,255,255]))

		mask = cv2.bitwise_or(cv2.bitwise_or(red_1, red_2), blue)

		#newmsg = msg
		#newmsg = self.bridge.cv2_to_imgmsg(mask)
		#self.pub_masked.publish(newmsg)

		#crop to just the bottom chunk of the screen
		crop_img = mask[350:450, :]
		newmsg = self.bridge.cv2_to_imgmsg(crop_img)
		self.pub_masked.publish(newmsg)

		#calculate center of the region of interest (error)
		i = 0
		s = 0
		for row in crop_img:
			for col in range(len(row)):
				if row[col] == 255:
					i+=1
					s+=col
		center = s / (i+0.0001)
				
		#update error values
		self.prev_error = self.curr_error
		self.curr_error = center*2 / msg.width - 1
		del_t = np.float64(msg.header.stamp.secs - self.prev_msg.header.stamp.secs)
		self.total_error += self.curr_error * del_t
		self.delta_error = (self.curr_error - self.prev_error) / del_t

		print self.prev_error
		print self.curr_error
		print self.total_error
		print self.delta_error
		print del_t
		print "****"

		self.prev_msg = msg

		self.state_lock.release()


#publish processed image??