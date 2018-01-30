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

SPATIAL_STEP = 5
ANGLE_STEP = 1.0

# get the map
# map_msg = rospy.ServiceProxy('static_map', GetMap)().map
map_img = Image.open('../maps/sieg_floor3.pgm')
for i in range(0, 3200, SPATIAL_STEP):
  for j in range(0, 3200, SPATIAL_STEP):
    if map_img.getpixel((i, j)) < 5:
      print (i, j)

# get LIDAR scan
bag = rosbag.Bag('../bags/laser_scans/laser_scan1.bag', 'r')
lidar_ranges = []
for topic, msg, t in bag.read_messages(topics=['/scan']):
  lidar_ranges = msg.ranges
bag.close()

# generate particles on map
particles = []
weights = []

# create sensor model
# sensor_model = SensorModel(map_msg, particles, weights)
