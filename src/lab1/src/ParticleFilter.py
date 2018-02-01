#!/usr/bin/env python

#import HeatMapping as hm
import rospy 
import numpy as np
import time
import utils as Utils
import tf.transformations
import tf
from threading import Lock

from vesc_msgs.msg import VescStateStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped, Point, Quaternion, Pose

from ReSample import ReSampler
from SensorModel import SensorModel
from MotionModel import OdometryMotionModel, KinematicMotionModel

 
class ParticleFilter():

  def __init__(self):
    self.MAX_PARTICLES = int(rospy.get_param("~max_particles")) # The maximum number of particles
    self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles")) # The maximum number of particles to visualize

    ###self.MAX_PARTICLES /= 2
    ###self.MAX_VIZ_PARTICLES = 100

    self.particle_indices = np.arange(self.MAX_PARTICLES)
    self.particles = np.zeros((self.MAX_PARTICLES,3)) # Numpy matrix of dimension MAX_PARTICLES x 3
    self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES) # Numpy matrix containig weight for each particle

    self.state_lock = Lock() # A lock used to prevent concurrency issues. You do not need to worry about this







    
    # Use the 'static_map' service (launched by MapServer.launch) to get the map
    # Will be used to initialize particles and SensorModel
    # Store map in variable called 'map_msg'
    # YOUR CODE HERE
    map_msg = rospy.ServiceProxy('static_map', GetMap)().map
    #print "MSG:" + str(map_msg)
    
    
    
    
    
    
    
    

    # Globally initialize the particles
    self.initialize_global(map_msg)
   
    # Publish particle filter state
    self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 1) # Publishes the expected pose
    self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    self.pub_tf = tf.TransformBroadcaster() # Used to create a tf between the map and the laser for visualization
    self.pub_laser     = rospy.Publisher("/pf/viz/scan", LaserScan, queue_size = 1) # Publishes the most recent laser scan

    self.RESAMPLE_TYPE = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling
    self.resampler = ReSampler(self.particles, self.weights, self.state_lock)  # An object used for resampling

    self.sensor_model = SensorModel(map_msg, self.particles, self.weights, self.state_lock) # An object used for applying sensor model
    self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.sensor_model.lidar_cb, queue_size=1)
    
    self.MOTION_MODEL_TYPE = rospy.get_param("~motion_model", "kinematic") # Whether to use the odometry or kinematics based motion model
    ###self.MOTION_MODEL_TYPE = "odometry"
    if self.MOTION_MODEL_TYPE == "kinematic":
      self.motion_model = KinematicMotionModel(self.particles, self.state_lock) # An object used for applying kinematic motion model
      self.motion_sub = rospy.Subscriber(rospy.get_param("~motion_topic", "/vesc/sensors/core"), VescStateStamped, self.motion_model.motion_cb, queue_size=1)
    elif self.MOTION_MODEL_TYPE == "odometry":
      self.motion_model = OdometryMotionModel(self.particles, self.state_lock)# An object used for applying odometry motion model
      self.motion_sub = rospy.Subscriber(rospy.get_param("~motion_topic", "/vesc/odom"), Odometry, self.motion_model.motion_cb, queue_size=1)
    else:
      print "Unrecognized motion model: "+ self.MOTION_MODEL_TYPE
      assert(False)
    
    # Use to initialize through rviz. Check clicked_pose_cb for more info    
    self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose_cb, queue_size=1)

  # Initialize the particles to cover the map
  def initialize_global(self, map_msg):
    # YOUR CODE HERE
    #extra credit so ignore for now
    pass
    
  # Publish a tf between the laser and the map
  # This is necessary in order to visualize the laser scan within the map
  def publish_tf(self,pose):
  # Use self.pub_tf
  # YOUR CODE HERE
    self.pub_tf.sendTransform((pose[0], pose[1], 0),tf.transformations.quaternion_from_euler(0, 0, pose[2]),rospy.Time.now(),"laser","map")
    ###self.pub_tf.sendTransform((pose[0], pose[1], 0),(0,0,0,1),rospy.Time.now(),"laser","map")

  # Returns the expected pose given the current particles and weights
  def expected_pose(self):
  # YOUR CODE HERE
    return np.mean(self.particles, axis=0)
    #return np.matmul(self.particles, self.weights)
    #ret = [0,0,0]
    #for i in range(self.MAX_PARTICLES):
    #    ret += self.particles[i]*self.weights[i]
    #return ret
    
  # Callback for '/initialpose' topic. RVIZ publishes a message to this topic when you specify an initial pose using its GUI
  # Reinitialize your particles and weights according to the received initial pose
  # Remember to apply a reasonable amount of Gaussian noise to each particle's pose
  def clicked_pose_cb(self, msg):
    self.state_lock.acquire()
    
    # YOUR CODE HERE
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    t = tf.transformations.euler_from_quaternion(q)
    for i in range(self.MAX_PARTICLES):
       self.particles[i][0] = x + np.random.normal(0, 0.1)
       self.particles[i][1] = y + np.random.normal(0, 0.1)
       self.particles[i][2] = t[2] + np.random.normal(0, 0.3)
       self.weights[i] = 1.0 / float(self.MAX_PARTICLES)
    
    self.state_lock.release()
    
  # Visualize the current state of the filter
  # (1) Publishes a tf between the map and the laser. Necessary for visualizing the laser scan in the map
  # (2) Publishes the most recent laser measurement. Note that the frame_id of this message should be the child_frame_id of the tf from (1)
  # (3) Publishes a PoseStampedw message indicating the expected pose of the car
  # (4) Publishes a subsample of the particles (use self.MAX_VIZ_PARTICLES). 
  #     Sample so that particles with higher weights are more likely to be sampled.
  def visualize(self):
    self.state_lock.acquire()
    
    # YOUR CODE HERE
    expected_pose = self.expected_pose()
    #1
    self.publish_tf(expected_pose)

    #2
    lasermsg = self.sensor_model.last_laser
    lasermsg.header.frame_id = "laser"
    self.pub_laser.publish(self.sensor_model.last_laser)
    
    #3
    exp = PoseStamped()
    exp.header.stamp = rospy.Time.now()
    exp.header.frame_id = "map"
    q = tf.transformations.quaternion_from_euler(0, 0, expected_pose[2])
    exp.pose = Pose(Point(expected_pose[0], expected_pose[1], 0), Quaternion(*q))
    self.pose_pub.publish(exp)

    #4
    import time
    a = time.time()
    vizparts = PoseArray()
    vizparts.header.stamp = rospy.Time.now()
    vizparts.header.frame_id = "map"
    indices = range(self.particles.shape[0])
    picked_indices = np.random.choice(indices, 100, True, self.weights); #instead of len(self.particles)
    for i in picked_indices:
        p = self.particles[i]
        q = tf.transformations.quaternion_from_euler(0, 0, p[2])
        ps = Pose(Point(p[0], p[1], 0), Quaternion(*q))
        vizparts.poses.append(ps)
    self.particle_pub.publish(vizparts)
    print('Viz: ', time.time() - a)
    self.state_lock.release()
  
# Suggested main 
if __name__ == '__main__':
  rospy.init_node("particle_filter", anonymous=True) # Initialize the node
  pf = ParticleFilter() # Create the particle filter
  #hm.main()

  while not rospy.is_shutdown(): # Keep going until we kill it
    # Callbacks are running in separate threads
    if pf.sensor_model.do_resample: # Check if the sensor model says it's time to resample
      pf.sensor_model.do_resample = False # Reset so that we don't keep resampling
      
      # Resample
      if pf.RESAMPLE_TYPE == "naiive":
        pf.resampler.resample_naiive()
      elif pf.RESAMPLE_TYPE == "low_variance":

        pf.resampler.resample_low_variance()
      else:
        print "Unrecognized resampling method: "+ pf.RESAMPLE_TYPE      
      
      pf.visualize() # Perform visualization



