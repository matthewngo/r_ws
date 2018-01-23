#!/usr/bin/env python

import rospy
import numpy as np
import utils as Utils
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
    curr_pose = np.array([msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.orientation.y])
      
    if isinstance(self.last_pose, np.ndarray):
      
      # Compute the control from the msg and last_pose
      # YOUR CODE HERE
      # deterministic, get the control (delta_x, delta_y, delta_theta) from msg - last_pose
      # assuming rotation is around the y-axis and we're in a flat 2D world
      control = [curr_pose[0] - self.last_pose[0],
                 curr_pose[1] - self.last_pose[1],
                 curr_pose[2] - self.last_pose[2]]
      # adding noise
      control[0] += np.random.normal(0, 1)
      control[1] += np.random.normal(0, 1)
      control[2] += np.random.normal(0, 1)

    
      self.apply_motion_model(self.particles, control)

    self.last_pose = curr_pose
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    # YOUR CODE HERE
    for row in proposal_dist:
      row[0] += control[0]
      row[1] += control[1]
      row[2] += control[2]
    
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
      return

    if self.last_vesc_stamp is None:
      self.last_vesc_stamp = msg.header.stamp

    # Convert raw msgs to controls
    # Note that control = (raw_msg_val - offset_param) / gain_param
    # YOUR CODE HERE
    curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    curr_steering_angle = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
    dt = msg.header.stamp - self.last_vesc_stamp

    curr_speed += np.random.normal(0, 1)
    curr_steering_angle += np.random.normal(0, 1)
    
    self.apply_motion_model(proposal_dist=self.particles, control=[curr_speed, curr_steering_angle, dt])
    self.last_vesc_stamp = msg.header.stamp
    self.state_lock.release()
    
  def apply_motion_model(self, proposal_dist, control):
    # Update the proposal distribution by applying the control to each particle
    
    for row in proposal_dist:
      delta_x = control[0] * np.cos(row[2])
      delta_y = control[0] * np.sin(row[2])
      beta = control[1] / 2
      delta_theta = (control[0] / 0.25) * np.sin(2 * beta)
      row[0] += delta_x
      row[1] += delta_y
      row[2] += delta_theta
    
if __name__ == '__main__':
  pass
    
