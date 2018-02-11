#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock

from sensor_msgs.msg import Image

import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

# param for saving intervals
IMAGE_STEP = 120

class ImageProcessor:

	def __init__(self, state_lock=None):
		#initialize stuff
		if state_lock is None:
			self.state_lock = Lock()
		else:
			self.state_lock = state_lock

		self.bridge = CvBridge()

		self.counter = 0

		#error goes from -1 (all the way to the left side of the image) to +1 (all the way to the right side of the image)
		self.curr_error = 0
		self.total_error = 0
		self.delta_error = 0
		self.prev_error = 0

		self.prev_msg = None

		self.pub_masked = rospy.Publisher("/lf/viz/masked", Image, queue_size = 1)

		self.use_blue = True

	def image_cb(self, msg):
		self.state_lock.acquire()

		#the first time, just take in the message and return
		if(self.prev_msg == None):
			self.prev_msg = msg
			self.state_lock.release()
			return

		#image processing:
		#convert message to cv2
		im = self.bridge.imgmsg_to_cv2(msg)

		# code to extract pictures
		if self.counter % IMAGE_STEP == 0:
			cv2.imwrite("./checker_img_" + str(self.counter), im)
		counter = counter + 1

		#convert rgb to hsv
		im_hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
		
		#threshold red/blue hues; everything else is black
		if(self.use_blue):
			mask = cv2.inRange(im_hsv, np.array([90,100,100]), np.array([120,255,255]))
		else:
			red_1 = cv2.inRange(im_hsv, np.array([0,100,100]), np.array([10,255,255]))
			red_2 = cv2.inRange(im_hsv, np.array([169,100,100]), np.array([179,255,255]))
			mask = cv2.bitwise_or(red_1, red_2)

		#newmsg = msg
		#newmsg = self.bridge.cv2_to_imgmsg(mask)
		#self.pub_masked.publish(newmsg)

		#crop to just the bottom chunk of the screen
		crop_img = mask[275:450, :]
		newmsg = self.bridge.cv2_to_imgmsg(crop_img)
		self.pub_masked.publish(newmsg)

                #use template matching to obtain new path
                

		#calculate center of the region of interest (error)
		i = 0
		s = 0
		col_num = 0
		#crop_img[crop_img != 255] = 0
		res = np.nonzero(crop_img)
		center = float(sum(res[1]))/(len(res[1]) + 0.0001)

		"""
		for col in np.transpose(crop_img):
			amt = list(col.flatten()).count(255)
			i+=amt
			s+=col_num*amt
			col_num += 1
		center = s / (i+0.0001)
		"""		
		#update error values
		self.prev_error = self.curr_error
		self.curr_error = center*2 / msg.width - 1
		del_t = np.float64(msg.header.stamp.secs - self.prev_msg.header.stamp.secs)
		del_t += np.float64((msg.header.stamp.nsecs - self.prev_msg.header.stamp.nsecs)/1000000000.0)
		self.total_error += self.curr_error * del_t
		self.delta_error = (self.curr_error - self.prev_error) / del_t

		"""print self.prev_error
		print self.curr_error
		print self.total_error
		print self.delta_error
		print del_t
		print "****" """

		self.prev_msg = msg

		print("Error: %.2f %.2f %.5f" % (self.curr_error, self.total_error, self.delta_error))

		self.state_lock.release()

	def extrinsics(self, msg):
                rotation_matrix = None
		translation_vector = [0.254, -0.026, 0.198]

	def intrinsics(self, msg):
		K = [618.0400390625, 0.0, 321.1227722167969, 0.0, 618.6351318359375, 235.7403106689453, 0.0, 0.0, 1.0]
		K = np.array(K)
		K = np.reshape(K, (3, 3))

	def impose_templates(self, msg):
                #
		templates = None
		img = None
                # convolve?
		

#publish processed image??
