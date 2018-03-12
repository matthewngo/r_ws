#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.srv import GetMap
import math
import Utils
from sensor_msgs.msg import Image
from ImageProcessing import ImageProcessor
from nav_msgs.msg import Path

# points = [[(609, 593), (1788, 2010), (2000, 1978)]]
points = [[(593, 609), (2010, 1788), (1978, 2000)]]
# map_x = None
# map_y = None

PIXELS_TO_METERS = 0.02

# curr = points[0]
# nxt = points[1]

steer = 0


def dist(a,b):
	return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def dot(a,b):
	return a[0]*b[0] + a[1]*b[1]

def abs(pt):
	return math.sqrt(pt[0]**2 + pt[1]**2)

def point_on_line(point, start, end):
	ang1 = 1.0 * dot(point-start, end-start) / (abs(point-start) * abs(end-start))
	ang2 = 1.0 * dot(point-end, start-end) / (abs(point-end) * abs(start-end))
	return (0 <= ang1 <= 1) and (0 <= ang2 <= 1)

def error_from_line(point, ang, start, end):
	disterr = (end[1]-start[1])*point[0] - (end[0]-start[0])*point[1] + end[0]*start[1] - end[1]*start[0]
	disterr /= dist(start, end)
	return (disterr, angle_err(ang, start, end))

def angle_err(ang, start, end):
	delx = end[0]-start[0]
	dely = end[1]-start[1]
	anglerr = math.atan2(dely, delx)
	return ang - anglerr

def get_error(point, ang, nodes):
	#find which node you're closest to
	point = np.array(point)
	mindist = 99999999
	minnode = None
	for i in range(len(nodes)):
		nodes[i] = np.array(nodes[i])
		d = dist(point, nodes[i])
		if d < mindist:
			mindist = d
			minnode = i

	# print minnode, nodes[minnode]

	disterr = 0
	anglerr = 0

	i = minnode
	if i == 0:
		disterr, anglerr = error_from_line(point, ang, nodes[0], nodes[1])
		if not point_on_line(point, nodes[0], nodes[1]):
			disterr = dist(point, nodes[0])
		return (disterr, anglerr)
	if i == len(nodes)-1:
		disterr, anglerr = error_from_line(point, ang, nodes[i-1], nodes[i])
		if not point_on_line(point, nodes[i-1], nodes[i]):
			disterr = dist(point, nodes[i])
		return (disterr, anglerr)

	else:
		onprev = point_on_line(point, nodes[i-1], nodes[i])
		onnext = point_on_line(point, nodes[i], nodes[i+1])
		if onprev == onnext:
			disterr, anglerr = error_from_line(point, ang, nodes[i-1], nodes[i+1])
			disterr = dist(point, nodes[i])
			return (disterr, anglerr)
		elif onprev:
			disterr,anglerr = error_from_line(point, ang, nodes[i-1], nodes[i])
			if not point_on_line(point, nodes[i-1], nodes[i]):
				disterr = dist(point, nodes[i])
			return (disterr, anglerr)
		else:
			disterr,anglerr = error_from_line(point, ang, nodes[i], nodes[i+1])
			if not point_on_line(point, nodes[i], nodes[i]+1):
				disterr = dist(point, nodes[i])
			return (disterr, anglerr)


def visualize():
	if path_pub.get_num_connections() > 0:
		frame_id = 'map'
		pa = Path()
		pa.header = Utils.make_header(frame_id)
		pa.poses = []
		for leg in points:
			for n in leg:
				n2 = [n[0] * PIXELS_TO_METERS + map_x, n[1] * PIXELS_TO_METERS + map_y, 0]
				pa.poses.append(Utils.particle_to_posestamped(n2, str(frame_id)))
		path_pub.publish(pa)


def follower_cb(msg):
	global steer
	pose = [(msg.pose.position.x - map_x) / PIXELS_TO_METERS,
			(msg.pose.position.y - map_y) / PIXELS_TO_METERS,
			Utils.quaternion_to_angle(msg.pose.orientation)]

	# pose[2] = pose[2] * 180.0 / math.pi

	leg = 0

	dist, angle = get_error(pose[0:2], pose[2], points[leg])
	angle += np.pi
	angle = angle % (np.pi*2)
	angle -= np.pi

	print dist, angle

	steer = dist * 0.05
	
	if angle < -np.pi/2:
		steer = 0.4
	elif angle > np.pi/2:
		steer = -0.4



if __name__ == '__main__':
	global map_x
	global map_y
	global image_processor
	global path_pub
	global allowed
	rospy.init_node("follower", anonymous=True)

	pose_sub  = rospy.Subscriber("/pf/viz/inferred_pose",
			PoseStamped, follower_cb, queue_size=1)

	pub_drive = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",
		AckermannDriveStamped, queue_size = 1)

	image_processor = ImageProcessor(None)
	image_sub = rospy.Subscriber(rospy.get_param("~image_topic", "/camera/color/image_raw"), Image, image_processor.image_cb, queue_size=1)

	

	map_service_name = rospy.get_param("~static_map", "static_map")
	print("Getting map from service: ", map_service_name)
	rospy.wait_for_service(map_service_name)
	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map # The map, will get passed to init of sensor model
	map_x = map_msg.info.origin.position.x
	map_y = map_msg.info.origin.position.y

	path_pub = rospy.Publisher("/viz/plan", Path, queue_size = 1)

	visualize()

	while not rospy.is_shutdown():
		#drive forward with constant speed and lf.angle


		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"

		msg.drive.steering_angle = steer
		msg.drive.speed = 1
			
		pub_drive.publish(msg)

		
