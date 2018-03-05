#!/usr/bin/env python

import time
import sys
import rospy
import rosbag
import numpy as np
import utils as Utils

import torch
import torch.utils.data
from torch.autograd import Variable

from nav_msgs.srv import GetMap
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped

#import Trainer
import math
import random

def make_input_mppi(ret_buff, prior, after, control, dt=0.1):
    ret_buff[:, 0] = after[:, 0] - prior[:, 0]
    ret_buff[:, 1] = after[:, 1] - prior[:, 1]
    ret_buff[:, 2] = after[:, 2] - prior[:, 2]
    ret_buff[:, 3] = np.cos(after[:, 2])
    ret_buff[:, 4] = np.sin(after[:, 2])
    ret_buff[:, 5] = control[:, 0]
    ret_buff[:, 6] = control[:, 1]
    ret_buff[:, 7] = dt

class MPPIController:

  def __init__(self, T, K, sigma=0.5, _lambda=0.5):
    self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0))
    self.SPEED_TO_ERPM_GAIN   = float(rospy.get_param("/vesc/speed_to_erpm_gain", 4614.0))
    self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5304))
    self.STEERING_TO_SERVO_GAIN   = float(rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135))
    self.CAR_LENGTH = 0.33 

    self.last_pose = None
    # MPPI params
    self.T = T # Length of rollout horizon
    self.K = K # Number of sample rollouts
    self.sigma = sigma
    self._lambda = _lambda

    self.controls = torch.cuda.FloatTensor(T, 2).zero_()
    self.score_tensor = torch.cuda.FloatTensor(K).zero_()
    self.weights = torch.cuda.FloatTensor(K).zero_()
    self.poses = torch.cuda.FloatTensor(K, T + 1, 3).zero_()
    self.e_thingy = torch.cuda.FloatTensor(K, T, 2)

    self.nn_input = torch.cuda.FloatTensor(K, 8).zero_()

    self.goal = None # Lets keep track of the goal pose (world frame) over time
    self.lasttime = None

    self.poses_pixels = torch.cuda.LongTensor(K, 2).zero_()

    self.noisy_controls = torch.cuda.FloatTensor(K, 2).zero_()

    self.goal_tensor = torch.cuda.FloatTensor(3).zero_()

    # PyTorch / GPU data configuration
    # TODO
    # you should pre-allocate GPU memory when you can, and re-use it when
    # possible for arrays storing your controls or calculated MPPI costs, etc
    #model_name = rospy.get_param("~nn_model")
    self.model = torch.load("/home/nvidia/catkin_ws/src/lab3/src/test.out") #model_name
    self.model.cuda() # tell torch to run the network on the GPU
    self.dtype = torch.cuda.FloatTensor
    print("Loading: model\n")
    print("Model:\n",self.model)
    print("Torch Datatype:", self.dtype)

    # control outputs
    self.msgid = 0

    # visualization paramters
    self.num_viz_paths = 5
    if self.K < self.num_viz_paths:
        self.num_viz_paths = self.K

    # We will publish control messages and a way to visualize a subset of our
    # rollouts, much like the particle filter
    self.ctrl_pub = rospy.Publisher(rospy.get_param("~ctrl_topic", "/vesc/high_level/ackermann_cmd_mux/input/nav_0"),
            AckermannDriveStamped, queue_size=2)
    self.path_pub = rospy.Publisher("/mppi/paths", Path, queue_size = self.num_viz_paths)

    # Use the 'static_map' service (launched by MapServer.launch) to get the map
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map # The map, will get passed to init of sensor model
    self.map_info = map_msg.info # Save info about map for later use    
    print("Map Information:\n",self.map_info)

    self.map_x = map_msg.info.origin.position.x
    self.map_y = map_msg.info.origin.position.y

    # Create numpy array representing map for later use
    self.map_height = map_msg.info.height
    self.map_width = map_msg.info.width
    array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
    self.permissible_region = np.zeros_like(array_255, dtype=bool)
    self.permissible_region[array_255==0] = 1 # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
                                              # With values 0: not permissible, 1: permissible
    self.permissible_region = np.negative(self.permissible_region) # 0 is permissible, 1 is not
    self.permissible_region = self.permissible_region.astype(float)
    self.permissible_region = torch.cuda.FloatTensor(self.permissible_region)
                                              
    print("Making callbacks")
    self.goal_sub = rospy.Subscriber("/move_base_simple/goal",
            PoseStamped, self.clicked_goal_cb, queue_size=1)
    self.pose_sub  = rospy.Subscriber("/pf/viz/inferred_pose", #was /pf/ta/viz/inferred_pose
            PoseStamped, self.mppi_cb, queue_size=1)
    print("Done!")
    
  # TODO
  # You may want to debug your bounds checking code here, by clicking on a part
  # of the map and convincing yourself that you are correctly mapping the
  # click, and thus the goal pose, to accessible places in the map
  def clicked_goal_cb(self, msg):
    self.goal = np.array([msg.pose.position.x,
                          msg.pose.position.y,
                          Utils.quaternion_to_angle(msg.pose.orientation)])
    print("Current Pose: ", self.last_pose)
    print("SETTING Goal: ", self.goal)
    self.controls = torch.cuda.FloatTensor(T, 2).zero_()
    self.poses = torch.cuda.FloatTensor(K, T + 1, 3).zero_()
    self.controls = torch.cuda.FloatTensor(T, 2).zero_()
    self.score_tensor = torch.cuda.FloatTensor(K).zero_()
    self.weights = torch.cuda.FloatTensor(K).zero_()
    
  def running_cost(self, poses, goal, ctrl, prev_ctrl, noise, prev_noise, score_tensor):
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

    # transform poses to pixels for bounds checking
    self.poses_pixels[:, 0] = (poses[:,0] - self.map_x) / 0.02
    self.poses_pixels[:, 1] = (poses[:,1] - self.map_y) / 0.02


    # penalize poses farther from the goal
    self.goal_tensor[0] = goal[0]
    self.goal_tensor[1] = goal[1]
    score_tensor[:] += pose_cost * torch.sqrt(torch.sum(torch.pow(poses[:, :2] - self.goal_tensor[:2].repeat(K, 1), 2), 1))

    # angle penalty
    score_tensor[:] += 0.5 * pose_cost * torch.abs(torch.add(torch.fmod(torch.add(torch.abs(poses[:, 2] - self.goal_tensor[2]), math.pi), 2 * math.pi), -1 * math.pi))

    # apply penalty for non-smooth controls

    ctr_being_notsmooth_a = self._lambda * ctrl[0] * (1.0 / self.sigma) * noise[:, 0]
    ctr_being_notsmooth_b = self._lambda * ctrl[1] * (1.0 / self.sigma) * noise[:, 1]
    #ctr_being_notsmooth = torch.sqrt(ctr_being_notsmooth_a.pow(2) + ctr_being_notsmooth_b.pow(2))
    ctr_being_notsmooth = 0.7 * torch.abs(ctr_being_notsmooth_a) + 0.3 * torch.abs(ctr_being_notsmooth_b)
    #score_tensor[:] += not_smooth_cost * ctr_being_notsmooth

    # penalize reversing direction enough to prevent robot spasms
    # x_mult_shit = (prev_ctrl[0] + prev_noise[:, 0]) * (ctrl[0] + noise[:, 0])
    # torch.mul(x_mult_shit, -1, out=x_mult_shit)
    # x_mult_shit.clamp_(0, 1)
    # torch.ceil(x_mult_shit, out=x_mult_shit)

    #score_tensor[:] += reverse_cost * x_mult_shit

    # check if any particle is out of bounds and if so penalize HEAVILY
    score_tensor[:] += bounds_check * self.permissible_region[self.poses_pixels[:, 1], self.poses_pixels[:, 0]]

    # score_tensor[:] += control_cost * math.sqrt(ctrl[0]**2 + ctrl[1]**2)

    # score_tensor[:] += noise_cost * torch.abs(torch.sum(noise, 1))

  def mppi(self, init_pose, init_input):
    t0 = time.time()
    # Network input can be:
    #   0    1       2          3           4        5      6   7
    # xdot, ydot, thetadot, sin(theta), cos(theta), vel, delta, dt

    # MPPI should
    # generate noise according to sigma
    # combine that noise with your central control sequence
    # Perform rollouts with those controls from your current pose
    # Calculate costs for each of K trajectories
    # Perform the MPPI weighting on your calculatd costs
    # Scale the added noise by the weighting and add to your control sequence
    # Apply the first control values, and shift your control trajectory

    if np.sum(np.abs(init_pose - self.goal)) <= 0.01:
      return (0,0), self.poses.zero_()

    T = self.T
    K = self.K
    vel_noise = self.sigma
    steer_noise = 0.2 * self.sigma
    controls = self.controls
    e_thingy = self.e_thingy
    # velocity noise
    e_thingy[:, :, 0] = torch.normal(std=torch.Tensor((([vel_noise] * K) * T)))
    e_thingy[:, :, 1] = torch.normal(std=torch.Tensor((([steer_noise] * K) * T)))
    score_tensor = self.score_tensor.zero_()
    weights = self.weights.zero_()
    poses = self.poses.zero_()
    poses[:, 0, :] = torch.cuda.FloatTensor(init_pose).repeat(K, 1)
    self.nn_input = init_input.repeat(K, 1)

    #wrap init pose around theta
    poses[:, 0, 2].add_(math.pi)
    poses[:, 0, 2].fmod_(2 * math.pi)
    poses[:, 0, 2].add_(-1 * np.pi)

    for t in range(1, T + 1):
      self.noisy_controls[:, 0] = e_thingy[:, t-1, 0] + controls[t-1, 0]
      self.noisy_controls[:, 1] = e_thingy[:, t-1, 1] + controls[t-1, 1]
      # noisy_controls = (torch.cuda.FloatTensor([] * K) + e_thingy[:, t-1, 0], torch.cuda.FloatTensor([controls[t-1, 1]] * K) + e_thingy[:, t-1, 1])
      if t > 1:
        prior = poses[:, t-2,:]
      elif t == 1:
        prior = poses[:, t-1,:]

      #old version
      #make_input_mppi(self.nn_input, prior, poses[:, t-1,:], self.noisy_controls) 
      #torch.add(self.model(Variable(self.nn_input)).data, value=1, other = poses[:, t-1, :], out = poses[:, t, :])

      #new version
      make_input_mppi(self.nn_input, prior, poses[:, t-1,:], self.noisy_controls)
      poses[:, t, :] = self.model(Variable(self.nn_input)).data
      poses[:, t, :] += prior

      #wrap around theta
      poses[:, t, 2].add_(math.pi)
      poses[:, t, 2].fmod_(2 * math.pi)
      poses[:, t, 2].add_(-1 * np.pi)

      # TODO: Replace with actual cost function and figure this shit out
      self.running_cost(poses[:, t, :], self.goal, controls[t - 1, :], controls[t - 2, :], e_thingy[:, t-1, :], e_thingy[:, t-2, :], score_tensor)
      # for k in range(K):
        #print "t = ", t
        # noisy_controls = (controls[t-1,0] + e_thingy[k, t-1, 0], controls[t-1, 1] + e_thingy[k, t-1, 1])

        # prior = poses[k, t-1,:]
        # if t > 1:
        #   prior = poses[k, t-2,:]

        # make_input_mppi(self.nn_input, prior, poses[k, t-1,:], noisy_controls) 
        # torch.add(self.model(Variable(self.nn_input)).data, value=1, other = poses[k, t-1, :], out = poses[k, t, :])

        # # TODO: Replace with actual cost function and figure this shit out
        # score_tensor[k] += self.running_cost(poses[k, t, :], self.goal, controls[t - 1, :], e_thingy[k, t - 1, :])

    #print score_tensor
    beta = torch.min(score_tensor)
    print 'beta,', beta
    norm = torch.sum(torch.exp((-1.0 / self._lambda) * (score_tensor - beta)))
    weights = (1.0 / norm) * torch.exp((-1.0 / self._lambda) * (score_tensor - beta))
    #controls[:, 0] += torch.sum(weights * e_thingy[K, :, 0])
    #controls[:, 1] += torch.sum(weights * e_thingy[K, :, 1])

    # repeat the weights along time
    weights_r = torch.t(weights.repeat(T, 1))
    # apply to controls
    controls[:, 0] += torch.sum(torch.mul(weights_r, e_thingy[:, :, 0]), 0)
    controls[:, 1] += torch.sum(torch.mul(weights_r, e_thingy[:, :, 1]), 0)


    #clamp controls - MAY NOT BE A GOOD IDEA I'M JUST TRYING STUFF
    # nah it's a good idea i did this last night before the robot nuked my code
    # just changed to clamp_ so it does it inplace
    controls[:,0].clamp_(-3, 3)
    controls[:,1].clamp_(-0.5, 0.5)

    # print self.controls

    # re add this when we actually run it
    run_ctrl = (controls[0, 0], controls[0, 1])
    controls[:-1, :] = controls[1:, :]
    controls[-1, 0] = 0
    controls[-1, 1] = 0


    # Notes:
    # MPPI can be assisted by carefully choosing lambda, and sigma
    # It is advisable to clamp the control values to be within the feasible range
    # of controls sent to the Vesc
    # Your code should account for theta being between -pi and pi. This is
    # important.
    # The more code that uses pytorch's cuda abilities, the better; every line in
    # python will slow down the control calculations. You should be able to keep a
    # reasonable amount of calculations done (T = 40, K = 2000) within the 100ms
    # between inferred-poses from the particle filter.

    print("MPPI: %4.5f ms" % ((time.time()-t0)*1000.0))

    #print run_ctrl

    return run_ctrl, poses

  def mppi_cb(self, msg):
    print("callback")
    if self.last_pose is None:
      self.last_pose = np.array([msg.pose.position.x,
                                 msg.pose.position.y,
                                 Utils.quaternion_to_angle(msg.pose.orientation)])
      # Default: initial goal to be where the car is when MPPI node is
      # initialized
      self.goal = self.last_pose
      self.lasttime = msg.header.stamp.to_sec()
      return

    theta = Utils.quaternion_to_angle(msg.pose.orientation)
    curr_pose = np.array([msg.pose.position.x,
                          msg.pose.position.y,
                          theta])

    pose_dot = curr_pose - self.last_pose # get state
    self.last_pose = curr_pose

    timenow = msg.header.stamp.to_sec()
    dt = timenow - self.lasttime
    self.lasttime = timenow
    # TODO: fix this shit
    nn_input = torch.cuda.FloatTensor(np.array([pose_dot[0], pose_dot[1], pose_dot[2],
                         np.sin(theta),
                         np.cos(theta), 0.0, 0.0, 0.1]))

    run_ctrl, poses = self.mppi(curr_pose, nn_input)

    self.send_controls( run_ctrl[0], run_ctrl[1] )

    self.visualize(poses)
  
  def send_controls(self, speed, steer):
    print("Speed:", speed, "Steering:", steer)
    ctrlmsg = AckermannDriveStamped()
    ctrlmsg.header.seq = self.msgid
    ctrlmsg.drive.steering_angle = steer 
    ctrlmsg.drive.speed = speed
    self.ctrl_pub.publish(ctrlmsg)
    self.msgid += 1

  # Publish some paths to RVIZ to visualize rollouts
  def visualize(self, poses):
    if self.path_pub.get_num_connections() > 0:
      frame_id = 'map'
      for i in range(0, self.num_viz_paths):
        pa = Path()
        pa.header = Utils.make_header(frame_id)
        pa.poses = []
        for pose in poses.cpu().numpy()[i,:,:]:
          pa.poses.append(Utils.particle_to_posestamped(pose, str(frame_id)))
        self.path_pub.publish(pa)

def test_MPPI(mp, N, goal=np.array([0.,0.,0.])):
  init_input = torch.cuda.FloatTensor(np.array([0.,0.,0.,0.,1.,0.,0.,0.]))
  pose = torch.cuda.FloatTensor(np.array([0.,0.,0.]))
  mp.goal = torch.cuda.FloatTensor(goal)
  print("Start:", pose)
  mp.controls.zero_()
  last_pose = torch.cuda.FloatTensor(np.array([0.,0.,0.]))
  for i in range(0,N):
    # ROLLOUT your MPPI function to go from a known location to a specified
    # goal pose. Convince yourself that it works.
    run_ctrl, poses = mp.mppi(last_pose, init_input)
    print run_ctrl
    #print poses
    # print("Now:", pose)
  print("End:", pose)
     
if __name__ == '__main__':

  T = 75 
  K = 1000
  sigma = 1.5 # These values will need to be tuned
  _lambda = 0.85

  # run with ROS
  rospy.init_node("mppi_control", anonymous=True) # Initialize the node
  mp = MPPIController(T, K, sigma, _lambda)
  rospy.spin()

  # test & DEBUG
  #mp = MPPIController(T, K, sigma, _lambda)
  #test_MPPI(mp, 50, np.array([10.,0.,0.]))


