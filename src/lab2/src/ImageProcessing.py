#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock



class ImageProcessor:

	def __init__(self, state_lock=None):
		#initialize stuff
		if state_lock is None:
			self.state_lock = Lock()
		else:
			self.state_lock = state_lock

	def image_cb(self, msg):
		self.state_lock.acquire()

		#do image processing

		self.state_lock.release()

	def get_error(self):
		#return the current error for the processed image

#publish processed image??