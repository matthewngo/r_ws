#!/usr/bin/env python

import rospy
import numpy as np
from  scipy import ndimage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Filter:
	def __init__(self, filter_path, sub_topic, pub_topic, fast_convolve=False):
		# Load the filter from csv
		filter_from_csv = np.loadtxt(filter_path, delimiter=',', ndmin=2)

		# Create the publisher and subscriber
		pub = rospy.Publisher(pub_topic, Image, queue_size=10)
		sub = rospy.Subscriber(sub_topic, Image, self.apply_filter_cb, queue_size=10)

		# Create a CvBridge object for converting sensor_msgs/Image into numpy arrays (and vice-versa)
		#		http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		# Use the 'self' parameter to initialize as necessary
                self.filter_array = filter_from_csv
                self.pub = pub
		self.bridge = CvBridge()
		self.fast_convolve = fast_convolve

	def apply_filter_cb(self, msg):
		# Apply the filter to the incoming image and publish the result
		cv = self.bridge.imgmsg_to_cv2(msg)
		result_cv = np.zeros([cv.shape[0], cv.shape[1], cv.shape[2]])
                
                # Fast convolve method
                if self.fast_convolve:
			result_cv[:,:,0] = ndimage.filters.convolve(cv[:,:,2], self.filter_array)
			result_cv[:,:,1] = ndimage.filters.convolve(cv[:,:,1], self.filter_array)
			result_cv[:,:,2] = ndimage.filters.convolve(cv[:,:,0], self.filter_array)
		# For loop method
                else:
			for i in range(0, cv.shape[0] - self.filter_array.shape[0]):
				for j in range(0, cv.shape[1] - self.filter_array.shape[1]):
					total = [0, 0, 0]
					for x in range(0, self.filter_array.shape[0]):
						for y in range(0, self.filter_array.shape[1]):
                                                        total[:] += cv[i+x, j+y, :] * self.filter_array[x,y]
                                        result_cv[i+1, j+1, :] = total[:]
					
		result_cv = result_cv.astype(np.uint8)
		result_msg = self.bridge.cv2_to_imgmsg(result_cv)

		# If the image has multiple channels, apply the filter to each channel independent of the other channels
		self.pub.publish(result_msg)
		
if __name__ == '__main__':
	filter_path = rospy.get_param("filter_path") #The path to the csv file containing the filter
	sub_topic = rospy.get_param("subscriber") # The image topic to be subscribed to
	pub_topic = rospy.get_param("publisher") # The topic to publish filtered images to
	fast_convolve = rospy.get_param("fast_convolve") # Whether or not the nested for loop or Scipy's convolve method should be used for applying the filter

	rospy.init_node('apply_filter', anonymous=True)
	
	# Populate params with values passed by launch file

	# Create a Filter object and pass it the loaded parameters
	f = Filter(filter_path, sub_topic, pub_topic, fast_convolve)

	rospy.spin()
	
