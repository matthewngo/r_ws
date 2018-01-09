#!/usr/bin/env python

import rospy
import numpy as np
from scipy import signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Filter:
	def __init__(self, filter_path, sub_topic, pub_topic, fast_convolve=False):
		# Load the filter from csv
		# Create the publisher and subscriber
		# Create a CvBridge object for converting sensor_msgs/Image into numpy arrays (and vice-versa)
		#		http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		# Use the 'self' parameter to initialize as necessary
		pass

	def apply_filter_cb(self, msg):
		# Apply the filter to the incoming image and publish the result
		# If the image has multiple channels, apply the filter to each channel independent of the other channels
		pass
		
if __name__ == '__main__':
	filter_path = None #The path to the csv file containing the filter
	sub_topic = None # The image topic to be subscribed to
	pub_topic = None # The topic to publish filtered images to
	fast_convolve = False # Whether or not the nested for loop or Scipy's convolve method should be used for applying the filter

	rospy.init_node('apply_filter', anonymous=True)
	
	# Populate params with values passed by launch file

	# Create a Filter object and pass it the loaded parameters

	rospy.spin()
	
