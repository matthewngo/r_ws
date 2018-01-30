#! /usr/bin/env python

from PIL import Image
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

# should be in meters
SPATIAL_STEP = 1
# should be in radians
ANGLE_STEP = 120
PIXELS_TO_METERS = 0.02

# get the map
map_msg = rospy.ServiceProxy('static_map', GetMap)().map
map_img = Image.open('../maps/sieg_floor3.pgm')
print map_img.width
'''

# generate particles from open locations on map
particles = []
for i in range(0, 3200, SPATIAL_STEP):
  for j in range(0, 3200, SPATIAL_STEP):
    if map_img.getpixel((i, j)) > 250:
      for k in range(0, 360, ANGLE_STEP):
        particles.append([i, j, k])
particles = np.array(particles)

# initialize weights to equal probability summing to one (1 / NUM_PARTICLES)
weights = np.ones(len(particles)) / len(particles)

# get LIDAR scan
bag = rosbag.Bag('../bags/laser_scans/laser_scan1.bag', 'r')
lidar_ranges = []
lidar_angles = []
for topic, msg, t in bag.read_messages(topics=['/scan']):
  lidar_ranges = msg.ranges
  for i in range(len(lidar_ranges)):
    lidar_angles.append(msg.angle_min + msg.angle_increment * i)
lidar_info = (lidar_ranges, lidar_angles)
bag.close()


# create and apply sensor model
sensor_model = SensorModel(map_msg, particles, weights)
sensor_model.precompute_sensor_model(max_range_px=280)
sensor_model.apply_sensor_model(particles, lidar_info, weights)
 
# combine heatmapping with original map and display
for i in range(len(particles)):
  particle = particles[i]
  weight = weights[i]
  map_img.putpixel((particle[0], particle[1]), int(weight * len(weights) * 60))

map_img.show()
'''
