#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.srv import GetMap
import math
import Utils

# points = [[(609, 593), (1788, 2010), (2000, 1978)]]
points = [[(593, 609), (2010, 1788), (1978, 2000)]]
# map_x = None
# map_y = None

PIXELS_TO_METERS = 0.02

# curr = points[0]
# nxt = points[1]

steer = 0


def point_to_line_dist(point, line):
	"""
	source: https://stackoverflow.com/questions/27161533/find-the-shortest-distance-between-a-point-and-line-segments-not-line

	Calculate the distance between a point and a line segment.

	To calculate the closest distance to a line segment, we first need to check
	if the point projects onto the line segment.  If it does, then we calculate
	the orthogonal distance from the point to the line.
	If the point does not project to the line segment, we calculate the 
	distance to both endpoints and take the shortest distance.

	:param point: Numpy array of form [x,y], describing the point.
	:type point: numpy.core.multiarray.ndarray
	:param line: list of endpoint arrays of form [P1, P2]
	:type line: list of numpy.core.multiarray.ndarray
	:return: The minimum distance to a point.
	:rtype: float
	"""
	# unit vector
	unit_line = line[1] - line[0]
	norm_unit_line = unit_line / np.linalg.norm(unit_line)

	# compute the perpendicular distance to the theoretical infinite line
	segment_dist = (
		np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
		np.linalg.norm(unit_line)
	)

	diff = (
		(norm_unit_line[0] * (point[0] - line[0][0])) + 
		(norm_unit_line[1] * (point[1] - line[0][1]))
	)

	x_seg = (norm_unit_line[0] * diff) + line[0][0]
	y_seg = (norm_unit_line[1] * diff) + line[0][1]

	endpoint_dist = min(
		np.linalg.norm(line[0] - point),
		np.linalg.norm(line[1] - point)
	)

	# decide if the intersection point falls on the line segment
	lp1_x = line[0][0]  # line point 1 x
	lp1_y = line[0][1]  # line point 1 y
	lp2_x = line[1][0]  # line point 2 x
	lp2_y = line[1][1]  # line point 2 y
	is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
	is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y
	if is_betw_x and is_betw_y:
		return segment_dist
	else:
		# if not, then return the minimum distance to the segment endpoints
		return endpoint_dist


def calc_path_error(path, pose):
	# convert pose to numpy array with [x, y]
	point = np.array([pose[0], pose[1]])

	# create list of lines from path
	lines = []
	for leg in path:
		for i in range(1, len(leg)):
			line = []
			line.append(np.array([leg[i - 1][0], leg[i - 1][1]]))
			line.append(np.array([leg[i][0], leg[i][1]]))
			lines.append(line)

	min_dist = float('inf')
	min_angle = 0.0
	for line in lines:
		dist = point_to_line_dist(point, line)
		if (dist < min_dist):
			min_dist = dist
			min_angle = np.rad2deg(np.arctan2(line[1][1] - line[0][1], line[1][0] - line[0][0])) - pose[2]

	return (min_dist, min_angle)



def follower_cb(msg):
	global steer
	pose = [(msg.pose.position.x - map_x) / PIXELS_TO_METERS,
			(msg.pose.position.y - map_y) / PIXELS_TO_METERS,
			Utils.quaternion_to_angle(msg.pose.orientation)]

	pose[2] = pose[2] * 180.0 / math.pi

	dist, angle = calc_path_error(points, pose)
	
	# dist = ((nxt[1] - curr[1]) * last_pose[0] - (nxt[0] - curr[0]) * last_pose[1] + nxt[0]*curr[1] - nxt[1]*curr[0]) / math.sqrt(((nxt[1] - curr[1])**2) + ((nxt[0] - curr[0]**2)))

	angle += 180
	angle = angle % 360
	angle -= 180

	if angle > 45:
		steer = 1
		print 'hard left'
	elif angle < -45:
		steer = -1
		print 'hard right'
	else:
		steer = 0
		print 'ehh'


if __name__ == '__main__':
	global map_x
	global map_y
	rospy.init_node("follower", anonymous=True)

	pose_sub  = rospy.Subscriber("/pf/viz/inferred_pose",
			PoseStamped, follower_cb, queue_size=1)

	pub_drive = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",
		AckermannDriveStamped, queue_size = 1)

	map_service_name = rospy.get_param("~static_map", "static_map")
	print("Getting map from service: ", map_service_name)
	rospy.wait_for_service(map_service_name)
	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map # The map, will get passed to init of sensor model
	map_x = map_msg.info.origin.position.x
	map_y = map_msg.info.origin.position.y
	

	while not rospy.is_shutdown():
		#drive forward with constant speed and lf.angle


		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"

		msg.drive.steering_angle = steer
		msg.drive.speed = 1
			
		pub_drive.publish(msg)