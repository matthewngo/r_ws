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
import torch
import MPPI

from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped

def main():
    print "start"
    # spatial discretization in pixels
    SPATIAL_STEP = 20 
    # angle discretization in degrees
    ANGLE_STEP = 10
    # resolution of map from pixels to meters
    PIXELS_TO_METERS = 0.02
    LIDAR_DOWNSAMPLING = 18
    ANGLE = math.pi / 2
    GOAL = torch.FloatTensor([-0.310417175293, -1.21807742119, 0])

    print 'getting map'
    # get the map
    map_msg = rospy.ServiceProxy('static_map', GetMap)().map
    print 'got map'

    map_matrix = np.reshape(map_msg.data, (map_msg.info.height, map_msg.info.width))
    map_img = Image.new('L', (map_msg.info.height, map_msg.info.width))
    for i in range(map_matrix.shape[0]):
      for j in range(map_matrix.shape[1]):
        map_img.putpixel((j, i), map_matrix[i][j])
       
    map_matrix = np.reshape(map_msg.data, (map_msg.info.height, map_msg.info.width))
    print 'height', map_msg.info.height
    print 'width', map_msg.info.width
    print 'getting particles'
    # generate particles from open locations on map
    particles = []
    for i in range(0, map_img.height, SPATIAL_STEP):
      for j in range(0, map_img.width, SPATIAL_STEP):
        if map_matrix[i][j] == 0:
          #for k in range(0, 360, ANGLE_STEP):
            particles.append([j, i, ANGLE])
    particles = torch.FloatTensor(particles)
    # np.savetxt('/home/mvn3/particles.csv', particles, delimiter=',', newline='\n')
    print 'got particles'

    # convert pixel values to meters for use in sensor model
    particles_meters = np.copy(particles)
    for i in range(particles_meters.shape[0]):
      particles_meters[i, 0] = particles_meters[i, 0] * PIXELS_TO_METERS + map_msg.info.origin.position.x
      particles_meters[i, 1] = particles_meters[i, 1] * PIXELS_TO_METERS + map_msg.info.origin.position.y
    particles_meters = torch.FloatTensor(particles_meters)

    score_tensor = torch.FloatTensor(particles.shape[0]).zero_()

    small_boy = score_tensor.min()

    # evaluate cost function on pose and goal
    running_cost(particles_meters, GOAL, score_tensor)
       
    # display cost map
    for i in range(particles.shape[0]):
        val = int(255 * (math.log(score_tensor[i]) + (1-math.log(small_boy))) / (1-math.log(small_boy)))
        map_img.putpixel((int(particles[i, 0]), int(particles[i, 1])), min(val, 0))
    
    map_img.save('out.png')

def running_cost(poses, goal, score_tensor):
    # TODO
    # This cost function drives the behavior of the car. You want to specify a
    # cost function that penalizes behavior that is bad with high cost, and
    # encourages good behavior with low cost.
    # We have split up the cost function for you to a) get the car to the goal
    # b) avoid driving into walls and c) the MPPI control penalty to stay
    # smooth
    # You should feel free to explore other terms to get better or unique
    # behavior
    pose_cost = 1000
    bounds_check = 999999999
    not_smooth_cost = 100

    # penalize poses farther from the goal
    score_tensor[:] += pose_cost * torch.sqrt(torch.sum(torch.pow(poses[:, :2] - goal[:2].repeat(poses.shape[0], 1), 2), 1))

    # angle penalty
    score_tensor[:] += 0.5 * pose_cost * torch.abs(torch.add(torch.fmod(torch.add(torch.abs(poses[:, 2] - goal[2]), math.pi), 2 * math.pi), -1 * math.pi))

main()