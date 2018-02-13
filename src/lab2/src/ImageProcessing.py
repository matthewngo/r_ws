#!/usr/bin/env python

import time
import rospy 
import numpy as np
from threading import Lock

from sensor_msgs.msg import Image

import cv2
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

from os import listdir

import tf.transformations
import tf

servo_min_angle = -0.263370416152
servo_max_angle = 0.31347342398

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
		print "yo"

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
		#cv2.imwrite('/home/mvn3/grayscale.png', crop_img)
		#return

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



class TemplateMatcher:

	def __init__(self, state_lock=None):
		#initialize stuff
		if state_lock is None:
			self.state_lock = Lock()
		else:
			self.state_lock = state_lock

		self.bridge = CvBridge()
		self.use_blue = False
		self.visible = False

		self.img = None

		self.templates = {}
		self.angles = []
		for f in listdir("/home/nvidia/catkin_ws/src/lab2/src/good_rollouts/"):
			if f == "all.png":
				continue
			angle = float(f[:-4])
			self.angles.append(float(angle))
			#load cv2 image
			im = cv2.imread("/home/nvidia/catkin_ws/src/lab2/src/good_rollouts/"+f, 0)

			#crop it
			#im = im[65:535, 105:720]

			#resize it
			#im = cv2.resize(im, (640,175))
			#im = im[50:,:]

			#make it into a mask
			#im = cv2.cvtColor(im, cv2.COLOR_GRAY2RGB)
			#im = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
			#im = cv2.inRange(im, np.array([0,0,0]), np.array([179,255,245]))
			self.templates[angle] = (im, np.nonzero(im))

		self.by_endpoint = np.zeros((60, len(self.templates)), dtype=object)
		self.angles = sorted(self.angles)
		i = 0
		while i < 60:
			for a in self.templates:
				t, lst = self.templates[a]
				if len(lst[0]) <= i:
					self.by_endpoint[i, self.angles.index(float(a))] = (-1, -1)
					continue
				self.by_endpoint[i, self.angles.index(float(a))] = (lst[0][i], lst[1][i])
			i += 1

		self.points_to_angle = {}
		for template_angle in self.templates:
			for point_idx in range(len(self.templates[template_angle][1][0])):
				point = (self.templates[template_angle][1][0][point_idx], self.templates[template_angle][1][1][point_idx])
				self.points_to_angle[point] = template_angle

		self.pub_masked = rospy.Publisher("/lf/viz/masked", Image, queue_size = 1)
		self.last_angle = 0

	print "good job"


	def image_cb(self, msg):
		self.state_lock.acquire()
		self.img = msg
		self.state_lock.release()

	def choose_template(self):
		if self.img == None:
			return 0

		self.state_lock.acquire()
		#process image
		im = self.bridge.imgmsg_to_cv2(self.img)
		im_hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
		if(self.use_blue):
			extracted = cv2.inRange(im_hsv, np.array([90,100,100]), np.array([120,255,255]))
		else:
			red_1 = cv2.inRange(im_hsv, np.array([0,100,100]), np.array([10,255,255]))
			red_2 = cv2.inRange(im_hsv, np.array([160,100,100]), np.array([179,255,255]))
			extracted = cv2.bitwise_or(red_1, red_2)

		extracted[0:200, :] = 0
		extracted[450:, :] = 0

		newmsg = self.bridge.cv2_to_imgmsg(extracted)
		self.pub_masked.publish(newmsg)

		# crop_img = extracted[275:450, :] #275:4

		img_sav = 0
		# Iterate through layers of each template
		for i in range(self.by_endpoint.shape[0]):
			row = self.by_endpoint[i, :]
			onion_layer = np.zeros([480, 640], dtype=np.uint8)
			for r, c in row:
				onion_layer[r, c] = 255
			dot_im = cv2.bitwise_and(onion_layer, extracted)
			px = np.nonzero(dot_im)
			if len(px[0]) == 0:
				continue
			r = px[0][0]
			c = px[1][0]
			angle = self.points_to_angle.get((r, c), None)
			if angle is None:
				continue
			# save template chosen overlaid on image
			#template_im = self.templates[angle][0]
			#out_im = cv2.add(extracted, template_im)
			#filename = "sample" + str(img_sav) + "_" + str(angle) + ".png"
			#cv2.imwrite("/home/nvidia/catkin_ws/src/lab2/src/sample_matching/"+filename, out_im) 
			#img_sav = img_sav + 1
			print angle
			self.visible = True
			self.state_lock.release()
			if angle > servo_max_angle:
				angle = servo_max_angle
			elif angle < servo_min_angle:
				angle = servo_min_angle
			self.last_angle = angle
			return angle

		self.visible = False
		self.state_lock.release()
		print 'no template matched', self.last_angle
		return self.last_angle

		# if len(np.nonzero(extracted)[1]) == 0:
		# 	self.visible = False
		# 	self.state_lock.release()
		# 	return 0
		# else:
		# 	self.visible = True

		# #compare with templates
		# bestangle = 0
		# bestcomp = 0
		# for a in self.templates:
		# 	template, points_list = self.templates[a]
		# 	for i in range(len(points_list[0])):
		# 		r, c = (points_list[0, i], points_list[1, i])
		# 		image_strip = extracted[r, :]


		# 	# comp = cv2.bitwise_and(extracted, template)
		# 	# overlap = len(np.nonzero(comp)[1])
		# 	# if overlap > bestcomp:
		# 	# 	bestangle = a
		# 	# 	bestcomp = overlap

		# #pick control
		# angle = bestangle
		# print "Angle: " + str(angle*57.295779513) + " " + str(bestcomp)
		
		# self.state_lock.release()
		# return angle
