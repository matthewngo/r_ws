#!/usr/bin/env python

import rospy
import math
import numpy as np
import utils as Utils
import tf.transformations
import tf
from std_msgs.msg import Float64
from threading import Lock

class OdometryMotionModel:

  def __init__(self, particles, state_lock=None):
    self.last_pose = None # The last pose that was received
    self.particles = particles
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
    
  def motion_cb(self, msg):
    self.state_lock.acquire()

    # initialize pose as np.ndarray with [x, y, theta]
    q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    t = tf.transformations.euler_from_quaternion(q)
    curr_pose = np.array([msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          t[2]])
      
    if isinstance(self.last_pose, np.ndarray):
      
      # Compute the control from the msg and last_pose
      # YOUR CODE HERE
      # deterministic, get the control (delta_x, delta_y, delta_theta) from msg - last_pose
      # assuming rotation is around the y-axis and we're in a flat 2D world
      control_diff = [curr_pose[0] - self.last_pose[0],
                     curr_pose[1] - self.last_pose[1]]
      rot_theta = self.last_pose[2]
      rot_matrix = [[math.cos(-rot_theta), -math.sin(-rot_theta)],[math.sin(-rot_theta), math.cos(-rot_theta)]]

      control = np.matmul(rot_matrix, control_diff)
      control = np.append(control, curr_pose[2] - self.last_pose[2])
      #print control
      self.apply_motion_model(self.particles, control)

    self.last_pose = curr_pose
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    # YOUR CODE HERE
    noise = 0.1 * np.random.randn(len(proposal_dist), 3)
    noisy_control = noise + control
    #print noisy_control
    sin_tab = np.sin(proposal_dist[:, 2])
    cos_tab = np.cos(proposal_dist[:, 2])

    proposal_dist[:, 0] = (cos_tab * noisy_control[:, 0]) - (sin_tab * noisy_control[:, 1])
    proposal_dist[:, 1] = (sin_tab * noisy_control[:, 0]) + (cos_tab * noisy_control[:, 1])
    proposal_dist[:, 2] = noisy_control[:, 2]
    
class KinematicMotionModel:

  def __init__(self, particles, state_lock=None):
    self.last_servo_cmd = None # The most recent servo command
    self.last_vesc_stamp = None # The time stamp from the previous vesc state msg
    self.particles = particles
    self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset")) # Offset conversion param from rpm to speed
    self.SPEED_TO_ERPM_GAIN   = float(rospy.get_param("/vesc/speed_to_erpm_gain"))   # Gain conversion param from rpm to speed
    self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset")) # Offset conversion param from servo position to steering angle
    self.STEERING_TO_SERVO_GAIN   = float(rospy.get_param("/vesc/steering_angle_to_servo_gain")) # Gain conversion param from servo position to steering angle
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
      
    # This subscriber just caches the most recent servo position command
    self.servo_pos_sub  = rospy.Subscriber(rospy.get_param("~servo_pos_topic", "/vesc/sensors/servo_position_command"), Float64,
                                       self.servo_cb, queue_size=1)
                                       

  def servo_cb(self, msg):
    self.last_servo_cmd = msg.data # Just update servo command

  def motion_cb(self, msg):
    self.state_lock.acquire()
    
    if self.last_servo_cmd is None:
      self.state_lock.release()
      return

    if self.last_vesc_stamp is None:
      self.last_vesc_stamp = msg.header.stamp

    # Convert raw msgs to controls
    # Note that control = (raw_msg_val - offset_param) / gain_param
    # YOUR CODE HERE
    curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    curr_steering_angle = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
    dt = (msg.header.stamp - self.last_vesc_stamp).to_sec()
    
    self.apply_motion_model(proposal_dist=self.particles, control=[curr_speed, curr_steering_angle, dt])
    self.last_vesc_stamp = msg.header.stamp
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    import time
    a = time.time()
    

    proposal_cos = np.cos(proposal_dist[:, 2])
    proposal_sin = np.sin(proposal_dist[:, 2])

    x_noise = 0.1 * np.random.randn(len(proposal_dist))
    y_noise = 0.1 * np.random.randn(len(proposal_dist))
    beta_noise = 0.1 * np.random.randn(len(proposal_dist), 1)
    theta_noise = 0.1 * np.random.randn(len(proposal_dist), 1)

    delta_x = np.multiply((x_noise + control[0]), proposal_cos) * control[2]
    delta_y = np.multiply((x_noise + control[0]), proposal_sin) * control[2]

    beta = np.arctan(0.5 * np.tan(beta_noise + control[1]))

    delta_theta = np.multiply(((theta_noise + control[0]) / 0.33), np.sin(2 * beta)) * control[2]

    print delta_x.shape
    print delta_y.shape
    print delta_theta.shape

    thing_to_add = np.column_stack([delta_x, delta_y, delta_theta])
    print thing_to_add.shape

    proposal_dist[:,:] = np.add(proposal_dist, thing_to_add)

    print('Motion: ', time.time()-a)
    
if __name__ == '__main__':
  pass
    
