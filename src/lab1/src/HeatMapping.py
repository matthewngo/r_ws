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
    # spatial discretization in pixels
    SPATIAL_STEP = 5
    # angle discretization in degrees
    ANGLE_STEP = 10
        # resolution of map from pixels to meters
    PIXELS_TO_METERS = 0.02
        LIDAR_DOWNSAMPLING = 18

        print 'getting map'
    # get the map
    map_msg = rospy.ServiceProxy('static_map', GetMap)().map
    print 'got map'

        map_matrix = np.reshape(map_msg.data, (map_msg.info.height, map_msg.info.width))
        map_img = Image.new('L', (map_msg.info.height, map_msg.info.width))
        for i in range(map_matrix.shape[0]):
          for j in range(map_matrix.shape[1]):
            map_img.putpixel((j, i), map_matrix[i][j])
        print 'showing image'
       
        map_matrix = np.reshape(map_msg.data, (map_msg.info.height, map_msg.info.width))
        print 'height', map_msg.info.height
        print 'width', map_msg.info.width
    print 'getting particles'
    # generate particles from open locations on map
    particles = []
    for i in range(0, map_img.height, SPATIAL_STEP):
      for j in range(0, map_img.width, SPATIAL_STEP):
        if map_matrix[i][j] == 0:
          for k in range(0, 360, ANGLE_STEP):
        particles.append([j, i, (k / 180.0) * math.pi])
                print map_img.getpixel((j, i))
    particles = np.array(particles)
        np.savetxt('/home/mvn3/particles.csv', particles, delimiter=',', newline='\n')
    print 'got particles'
       
    # initialize weights to equal probability summing to one (1 / NUM_PARTICLES)
    weights = np.ones(len(particles)) / len(particles)
   
    print 'getting scan'
    # get LIDAR scan
    bag = rosbag.Bag('/home/mvn3/r_ws/src/lab1/bags/laser_scans/laser_scan1.bag', 'r')
    lidar_ranges = []
    lidar_angles = []
    for topic, msg, t in bag.read_messages(topics=['/scan']):
      lidar_ranges = np.array(msg.ranges, dtype=np.float32)
          lidar_ranges = lidar_ranges[::LIDAR_DOWNSAMPLING]
          lidar_ranges = lidar_ranges.copy(order='C')
      for i in range(len(lidar_ranges)):
        lidar_angles.append(msg.angle_min + msg.angle_increment * i * LIDAR_DOWNSAMPLING)
    lidar_info = (lidar_ranges, np.array(lidar_angles, dtype=np.float32))
    bag.close()

        print lidar_info
    print 'got scan'

    # convert pixel values to meters for use in sensor model
    particles_meters = np.copy(particles)
    for i in range(len(particles_meters)):
      particles_meters[i][0] = particles_meters[i][0] * PIXELS_TO_METERS + map_msg.info.origin.position.x
      particles_meters[i][1] = particles_meters[i][1] * PIXELS_TO_METERS + map_msg.info.origin.position.y
        particles_meters = np.array(particles_meters, dtype=np.float32)
    print 'applying sensor model'
       
    # create and apply sensor model
    sensor_model = SensorModel(map_msg, particles_meters, weights)
        print 'init done, sum weights:', np.sum(weights)
    sensor_model.apply_sensor_model(particles_meters, lidar_info, weights)
    print 'applied sensor, sum weights:', np.sum(weights)
       

        print 'weights:'
    print np.sum(weights)
       
        # renormalize weights
        weights = weights / np.sum(weights)
        print np.sum(weights)
       
        last_particle = particles[0]
        max_weight = weights[0]
        small_boy = np.amin(weights)
        weight_factor = 1/float(small_boy)
    # combine heatmapping with original map and display
    for i in range(1, len(particles)):
      particle = particles[i]
      weight = weights[i]

          if (last_particle[0] != particle[0] or last_particle[1] != particle[1]):
            # pixel_value = int(255 * (math.log(max_weight) + (1-math.log(small_boy))) / (1-math.log(small_boy)))
            pixel_value = 255 * max_weight
            #assert pixel_value < 256 and pixel_value >= 0
            if (max_weight > 0.001):
              print last_particle[0], last_particle[1], last_particle[2]
              print max_weight
            if pixel_value <= 255:
              map_img.putpixel((int(last_particle[0]), int(last_particle[1])), pixel_value)
            last_particle = particle
            max_weight = weight
          else:
            if weight > max_weight:
              max_weight = weight
            last_particle = particle
    print 'end'
    map_img.show()


