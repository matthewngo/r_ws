#!/usr/bin/env python

import numpy as np
#from scipy import stats
import rospy
import range_libc
import time
from threading import Lock
import math

THETA_DISCRETIZATION = 112 # Discretization of scanning angle
INV_SQUASH_FACTOR = 2.2    # Factor for helping the weight distribution to be less peaked

Z_SHORT = 0.1  # Weight for short reading
Z_MAX = 0.1    # Weight for max reading
Z_RAND = 0.05   # Weight for random reading
SIGMA_HIT = 1 # Noise value for hit reading
Z_HIT = 0.75    # Weight for hit reading

class SensorModel:
	
  def __init__(self, map_msg, particles, weights, state_lock=None):
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
    self.particles = particles
    self.weights = weights
    
    self.LASER_RAY_STEP = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
    self.MAX_RANGE_METERS = float(rospy.get_param("~max_range_meters")) # The max range of the laser
    
    oMap = range_libc.PyOMap(map_msg) # A version of the map that range_libc can understand
    max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution) # The max range in pixels of the laser
    self.range_method = range_libc.PyCDDTCast(oMap, max_range_px, THETA_DISCRETIZATION) # The range method that will be used for ray casting
    self.range_method.set_sensor_model(self.precompute_sensor_model(max_range_px)) # Load the sensor model expressed as a table
    self.queries = None
    self.ranges = None
    self.laser_angles = None # The angles of each ray
    self.downsampled_angles = None # The angles of the downsampled rays 
    self.do_resample = False # Set so that outside code can know that it's time to resample
    
  def lidar_cb(self, msg):
    self.state_lock.acquire()

    # Compute the observation
    # obs is a a two element tuple
    # obs[0] is the downsampled ranges
    # obs[1] is the downsampled angles
    # Each element of obs must be a numpy array of type np.float32 (this is a requirement for GPU processing)
    # Use self.LASER_RAY_STEP as the downsampling step
    # Keep efficiency in mind, including by caching certain things that won't change across future iterations of this callback
  
    # YOUR CODE HERE
    obs = [0, 0]
    obs[0] = np.array(msg.ranges[::self.LASER_RAY_STEP], dtype=np.float32)

    if self.downsampled_angles is None:
      self.downsampled_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment * self.LASER_RAY_STEP, np.float32)  

    obs[1] = self.downsampled_angles
    obs = tuple(obs)
 
    ###print(self.weights)
    self.apply_sensor_model(self.particles, obs, self.weights)
    ###print(self.weights)
    self.weights /= np.sum(self.weights)
    ###print(self.weights)
    ###print(" \n")
    
    self.last_laser = msg
    self.do_resample = True
    
    self.state_lock.release()
    
  def precompute_sensor_model(self, max_range_px):

    table_width = int(max_range_px) + 1
    sensor_model_table = np.zeros((table_width,table_width))

    # Populate sensor model table as specified
    # Note that the row corresponds to the observed measurement and the column corresponds to the expected measurement
    # YOUR CODE HERE 
    probs_normal = np.zeros([table_width, table_width])
    probs_expon = np.zeros([table_width, table_width])
    probs_max = np.zeros([table_width, table_width])
    probs_uniform = np.zeros([table_width, table_width], dtype=np.float64)
    for row in range(sensor_model_table.shape[0]):
      for column in range (sensor_model_table.shape[1]):
        probs_normal[row, column] = (1.0/math.sqrt(2*math.pi*0.5)) * math.exp((row-column)*(row-column)/(-2.0*0.5)) #stats.norm.pdf(row, column, 1)
        probs_expon[row, column] = 1.0 * math.exp(-1.0 * row) #stats.expon.pdf(row, 0, 1)
        if row == max_range_px:
          probs_max[row, column] = 1
        probs_uniform[row, column] = np.float64(1.0) / max_range_px

    probs_normal = probs_normal / probs_normal.sum(axis=0, keepdims=1)
    probs_expon = probs_expon / probs_expon.sum(axis=0, keepdims=1)

    wts = np.array([0.75, 0.1, 0.1, 0.05])

    for row in range(sensor_model_table.shape[0]):
      for column in range (sensor_model_table.shape[1]):
        sensor_model_table[row, column] = (probs_normal[row, column] * wts[0]) + (probs_expon[row, column] * wts[1]) + (probs_max[row, column] * wts[2]) + (probs_uniform[row, column] * wts[3])

    #print 'normal sums', probs_normal.sum(axis=0)
    #print 'expon sums', probs_expon.sum(axis=0)
    #print 'max sums', probs_max.sum(axis=0)
    #print 'uniform sums', probs_uniform.sum(axis=0)
    #print max_range_px
    return sensor_model_table

  def apply_sensor_model(self, proposal_dist, obs, weights):
        
    obs_ranges = obs[0]
    obs_angles = obs[1]
    num_rays = obs_angles.shape[0]
    
    # Only allocate buffers once to avoid slowness
    if not isinstance(self.queries, np.ndarray):
      self.queries = np.zeros((proposal_dist.shape[0],3), dtype=np.float32)
      self.ranges = np.zeros(num_rays*proposal_dist.shape[0], dtype=np.float32)

    self.queries[:,:] = proposal_dist[:,:]

    self.range_method.calc_range_repeat_angles(self.queries, obs_angles, self.ranges)

    ###print(weights)

    # Evaluate the sensor model on the GPU
    self.range_method.eval_sensor_model(obs_ranges, self.ranges, weights, num_rays, proposal_dist.shape[0])

    ###print(weights)

    np.power(weights, INV_SQUASH_FACTOR, weights)

if __name__ == '__main__':
  pass
