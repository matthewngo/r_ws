#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock

from sensor_msgs.msg import Image

import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

from os import listdir

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

		self.visible = False

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
		if self.counter % 3 == 0:
			print "yo", self.counter
			cv2.imwrite("/home/mvn3/checkboard_boys/checker_img_" + str(self.counter + 200) + ".png", im)
			print "done saving", self.counter
		print self.counter
		self.counter = self.counter + 1

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

		#use template matching to obtain new path

		#calculate center of the region of interest (error)
		#i = 0
		#s = 0
		#col_num = 0
		#crop_img[crop_img != 255] = 0
		res = np.nonzero(crop_img)
		center = float(sum(res[1]))/(len(res[1]) + 0.0001)
		if len(res[1]) == 0:
			self.visible = False
		else:
			self.visible = True

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
		if self.visible == False:
			self.curr_error = 0
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

		#print("Error: %.2f %.2f %.5f" % (self.curr_error, self.total_error, self.delta_error))

		#publish cropped image
		mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
		mask[0:275,:,2] = 0
		mask[450:480,:,2] = 0
		newmsg = self.bridge.cv2_to_imgmsg(mask)
		self.pub_masked.publish(newmsg)
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


	def rotation_matrix(angle, direction, point=None):
		sina = math.sin(angle)
		cosa = math.cos(angle)
		direction = unit_vector(direction[:3])
		# rotation matrix around unit vector
		R = numpy.diag([cosa, cosa, cosa])
		R += numpy.outer(direction, direction) * (1.0 - cosa)
		direction *= sina
		R += numpy.array([[ 0.0,-direction[2], direction[1]],
				[ direction[2], 0.0, -direction[0]],
				[-direction[1], direction[0], 0.0]])
		M = numpy.identity(4)
		M[:3, :3] = R
		if point is not None:
			# rotation not around origin
			point = numpy.array(point[:3], dtype=numpy.float64, copy=False)
			M[:3, 3] = point - numpy.dot(R, point)
		return M



class TemplateFollower:

	def __init__(self, state_lock=None):
		#initialize stuff
		if state_lock is None:
			self.state_lock = Lock()
		else:
			self.state_lock = state_lock

		self.bridge = CvBridge()
		self.use_blue = True
		self.visible = False

		self.img = None

		self.templates = []


	def image_cb(self, msg):
		self.state_lock.acquire()
		self.img = msg
		self.state_lock.release()