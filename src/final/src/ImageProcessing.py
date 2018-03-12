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

		self.prev_msg = None

		self.pub_red = rospy.Publisher("/lf/viz/masked_red", Image, queue_size = 1)
		self.pub_blue = rospy.Publisher("/lf/viz/masked_blue", Image, queue_size = 1)

		#dist goes from 0 (far) to 1 (close)
		#center goes from -1 (far left of FOV) to 1 (far right of FOV); if it isn't seen, center is 0 
		self.red_dist = 0
		self.red_center = 0
		self.red_prev = 0
		self.red_del = 0
		self.red_count = 0

		self.blue_dist = 0
		self.blue_center = 0
		self.blue_prev = 0
		self.blue_del = 0
		self.blue_count = 0


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

		#convert rgb to hsv
		im_hsv = cv2.cvtColor(im, cv2.COLOR_RGB2HSV)
		
		#threshold red/blue hues; everything else is black
		blue_mask = cv2.inRange(im_hsv, np.array([90,100,100]), np.array([120,255,255]))
		red_1 = cv2.inRange(im_hsv, np.array([0,100,100]), np.array([10,255,255]))
		red_2 = cv2.inRange(im_hsv, np.array([169,100,100]), np.array([179,255,255]))
		red_mask = cv2.bitwise_or(red_1, red_2)

		#crop to just the bottom chunk of the screen
		#TODO: actually test cropping
		crop_blue = blue_mask[275:450, :]
		crop_red = red_mask[275:450, :]

		del_t = np.float64(msg.header.stamp.secs - self.prev_msg.header.stamp.secs)
		del_t += np.float64((msg.header.stamp.nsecs - self.prev_msg.header.stamp.nsecs)/1000000000.0)

		### RED ###

		#calculate center of the region of interest (error)
		res = np.nonzero(crop_red)
		center = float(sum(res[1]))/(len(res[1]) + 0.0001)
		height = float(max(res[0]))
		self.red_count = len(res[1])

		#update error values
		self.red_prev = self.red_center
		self.red_center = center*2 / msg.width - 1
		self.red_dist = height / 175
		if self.red_count == 0:
			self.red_center = 0
			self.red_dist = 1
		self.red_del = (self.red_center - self.red_prev) / del_t

		#publish cropped image
		mask = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2RGB)
		mask[0:275,:,1] = 0
		mask[450:480,:,1] = 0
		newmsg = self.bridge.cv2_to_imgmsg(mask)
		self.pub_red.publish(newmsg)

		### BLUE ####

		#calculate center of the region of interest (error)
		res = np.nonzero(crop_blue)
		center = float(sum(res[1]))/(len(res[1]) + 0.0001)
		height = float(max(res[0]))
		self.blue_count = len(res[1]) != 0

		#update error values
		self.blue_prev = self.blue_center
		self.blue_center = center*2 / msg.width - 1
		self.blue_dist = height / 175
		if self.blue_count == 0:
			self.blue_center = 0
			self.blue_dist = 1
		self.blue_del = (self.blue_center - self.blue_prev) / del_t

		#publish cropped image
		mask = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2RGB)
		mask[0:275,:,2] = 0
		mask[450:480,:,2] = 0
		newmsg = self.bridge.cv2_to_imgmsg(mask)
		self.pub_blue.publish(newmsg)

		self.prev_msg = msg

		self.state_lock.release()

