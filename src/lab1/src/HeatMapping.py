#! /usr/bin/env python

from PIL import Image
import math
import rospy
import rosbag
import numpy as np
import time
import utils as Utils
import tf.transformations
import tf
from threading import Lock

from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped

from ReSample import ReSampler
from SensorModel import SensorModel
from MotionModel import OdometryMotionModel, KinematicMotionModel

def main():
	print "start"
	# should be in meters
	SPATIAL_STEP = 5
	# should be in radians
	ANGLE_STEP = 360
	PIXELS_TO_METERS = 0.02

        print 'getting map'
	# get the map
	map_msg = rospy.ServiceProxy('static_map', GetMap)().map
	map_img = Image.open('/home/mvn3/r_ws/src/lab1/maps/sieg_floor3.pgm')
	print 'got map'

	print 'getting particles'
	# generate particles from open locations on map
	particles = []
	for i in range(0, 3200, SPATIAL_STEP):
	  for j in range(0, 3200, SPATIAL_STEP):
	    if map_img.getpixel((i, j)) > 250:
	      for k in range(0, 360, ANGLE_STEP):
		particles.append([i, j, (k / 180.0) * math.pi])
	particles = np.array(particles)
	print 'got particles'

	# initialize weights to equal probability summing to one (1 / NUM_PARTICLES)
	weights = np.ones(len(particles)) / len(particles)
	
	print 'getting scan'
	# get LIDAR scan
	bag = rosbag.Bag('/home/mvn3/r_ws/src/lab1/bags/laser_scans/laser_scan1.bag', 'r')
	lidar_ranges = []
	lidar_angles = []
	for topic, msg, t in bag.read_messages(topics=['/scan']):
	  lidar_ranges = msg.ranges
	  for i in range(len(lidar_ranges)):
	    lidar_angles.append(msg.angle_min + msg.angle_increment * i)
	lidar_info = (np.array(lidar_ranges), np.array(lidar_angles))
	bag.close()
	print 'got scan'

	# convert pixel values to meters for use in sensor model
	particles_meters = particles[:]
	for i in range(len(particles_meters)):
	  particles_meters[i][0] = particles_meters[i][0] * PIXELS_TO_METERS
	  particles_meters[i][1] = particles_meters[i][1] * PIXELS_TO_METERS

	print 'applying sensor model'
	# create and apply sensor model
	sensor_model = SensorModel(map_msg, particles_meters, weights)
	sensor_model.precompute_sensor_model(max_range_px=280)
	sensor_model.apply_sensor_model(particles_meters, lidar_info, weights)
	print 'applied sensor model'

	# combine heatmapping with original map and display
	for i in range(len(particles)):
	  particle = particles[i]
	  weight = weights[i]
	  map_img.putpixel((particle[0], particle[1]), 255 - int(weight * len(weights) * 128))
	print 'end'
	map_img.show()

