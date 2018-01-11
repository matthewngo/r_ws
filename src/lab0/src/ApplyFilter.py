#!/usr/bin/env python

import rospy
import numpy as np
from scipy import signal
from  scipy import ndimage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Filter:
	def __init__(self, filter_path, sub_topic, pub_topic, fast_convolve=False):
		# Load the filter from csv
		filter_array = np.loadtxt(filter_path, delimiter=',', ndmin=2)
		# Create the publisher and subscriber
		publisher = rospy.Publisher(pub_topic, Image, queue_size=10)
		subscriber = rospy.Subscriber(sub_topic, Image, self.apply_filter_cb, queue_size=10)
		# Create a CvBridge object for converting sensor_msgs/Image into numpy arrays (and vice-versa)
		#		http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		bridge = CvBridge()
		# Use the 'self' parameter to initialize as necessary
		self.converter = bridge
		self.publisher = publisher
		self.filter_array = filter_array
		self.fast_convolve = fast_convolve

	def apply_filter_cb(self, msg):
		# Apply the filter to the incoming image and publish the result
		cv_img = self.converter.imgmsg_to_cv2(msg)
		result_cv_image = np.zeros([cv_img.shape[0], cv_img.shape[1], cv_img.shape[2]])
		if self.fast_convolve:
			result_cv_image[:,:,2] = ndimage.filters.convolve(cv_img[:,:,0], self.filter_array)
			result_cv_image[:,:,1] = ndimage.filters.convolve(cv_img[:,:,1], self.filter_array)
			result_cv_image[:,:,0] = ndimage.filters.convolve(cv_img[:,:,2], self.filter_array)
		else:
			for i in range(0, cv_img.shape[0] - self.filter_array.shape[0]):
				for j in range(0, cv_img.shape[1] - self.filter_array.shape[1]):
					total = [0, 0, 0]
					for x in range(0, self.filter_array.shape[0]):
						for y in range(0, self.filter_array.shape[1]):
							total[0] += cv_img[i+x, j+y, 0] * self.filter_array[x, y]
							total[1] += cv_img[i+x, j+y, 1] * self.filter_array[x, y]
							total[2] += cv_img[i+x, j+y, 2] * self.filter_array[x, y]
					result_cv_image[i+1, j+1, 0] = total[0]
					result_cv_image[i+1, j+1, 1] = total[1]
					result_cv_image[i+1, j+1, 2] = total[2]
					
					
		result_cv_image = result_cv_image.astype(np.uint8)
		result_msg = self.converter.cv2_to_imgmsg(result_cv_image)
		# If the image has multiple channels, apply the filter to each channel independent of the other channels
		self.publisher.publish(result_msg)
		
if __name__ == '__main__':
	filter_path = rospy.get_param('filter_path') #The path to the csv file containing the filter
	sub_topic = rospy.get_param('sub_topic') # The image topic to be subscribed to
	pub_topic = rospy.get_param('pub_topic') # The topic to publish filtered images to
	fast_convolve = rospy.get_param('fast_convolve') # Whether or not the nested for loop or Scipy's convolve method should be used for applying the filter

	rospy.init_node('apply_filter', anonymous=True)
	
	# Populate params with values passed by launch file

	# Create a Filter object and pass it the loaded parameters
	filter1 = Filter(filter_path, sub_topic, pub_topic, fast_convolve)

	rospy.spin()
	
