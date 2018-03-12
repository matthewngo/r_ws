import math
import numpy as np
import heapq
import tf

import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
import tf.transformations
import tf
import matplotlib.pyplot as plt

def pixels_between(p1, p2):
    # Bresenham's Line Algorithm
    # http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python

    # Setup initial conditions
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


# thanks cse 473
class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.

      Note that this PriorityQueue does not allow you to change the priority
      of an item.  However, you may insert the same item multiple times with
      different priorities.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        # FIXME: restored old behaviour to check against old results better
        # FIXED: restored to stable behaviour
        entry = (priority, self.count, item)
        # entry = (priority, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        #  (_, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

def mHeuristic(xy1, xy2):
    # return 0
    return abs(xy1[0] - xy2[0] ) + abs( xy1[1] - xy2[1] )


class PriorityQueueWithFunction(PriorityQueue):
    """
    Implements a priority queue with the same push/pop signature of the
    Queue and the Stack classes. This is designed for drop-in replacement for
    those two classes. The caller has to provide a priority function, which
    extracts each item's priority.
    """
    def  __init__(self, priorityFunction):
        "priorityFunction (item) -> priority"
        self.priorityFunction = priorityFunction      # store the priority function
        PriorityQueue.__init__(self)        # super-class initializer

    def push(self, item):
        "Adds an item to the queue with priority from the priority function"
        PriorityQueue.push(self, item, self.priorityFunction(item))

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw

def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])

def particle_to_posestamped(particle, frame_id):
    pose = PoseStamped()
    pose.header = make_header(frame_id)
    pose.pose.position.x = particle[0]
    pose.pose.position.y = particle[1]
    pose.pose.orientation = angle_to_quaternion(particle[2])
    return pose

def particle_to_pose(particle):
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = angle_to_quaternion(particle[2])
    return pose

def particles_to_poses(particles):
    return map(particle_to_pose, particles)

def make_header(frame_id, stamp=None):
    if stamp == None:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header

def point(npt):
    pt = Point32()
    pt.x = npt[0]
    pt.y = npt[1]
    return pt

def points(arr):
    return map(point, arr)

def map_to_world(poses,map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)

    # rotate

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:,0])
    poses[:,0] = c*poses[:,0] - s*poses[:,1]
    poses[:,1] = s*temp       + c*poses[:,1]

    # scale
    poses[:,:2] *= float(scale)

    # translate
    poses[:,0] += map_info.origin.position.x
    poses[:,1] += map_info.origin.position.y
    poses[:,2] += angle

def world_to_map(poses, map_info):
    # equivalent to map_to_grid(world_to_map(poses))
    # operates in place
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)

    # translation
    poses[:,0] -= map_info.origin.position.x
    poses[:,1] -= map_info.origin.position.y

    # scale
    poses[:,:2] *= (1.0/float(scale))

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:,0])
    poses[:,0] = c*poses[:,0] - s*poses[:,1]
    poses[:,1] = s*temp       + c*poses[:,1]
    poses[:,2] += angle

