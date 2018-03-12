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
# points = [[(593, 609), (2010, 1788), (1978, 2000)]]
points = [[(699, 620), (1034, 1070)],
[(1034, 1070), (1379, 1504), (1484, 1500)],
[(1484, 1500), (1591, 1758), (1888, 2032), (2094, 1900)],
[(2094, 1900), (1684, 2250)],
[(1684, 2250), (1764, 2316), (1826, 2427)]]

global points_flat
points_flat = None

PIXELS_TO_METERS = 0.02

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
	global off_end

	#find which node you're closest to
	mindist = 99999999
	minnode = None
	for i in range(len(nodes)):
		d = dist(point, nodes[i])
		if d < mindist:
			mindist = d
			minnode = i

	# print minnode, nodes[minnode]

	disterr = 0
	anglerr = 0

	off_end = False

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
		off_end = True
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
	global points_flat
	global leg 

	if points_flat == None:
		points_flat = []
		for l in points:
			for n in l:
				points_flat.append(n)

	if path_pub.get_num_connections() > 0:
		frame_id = 'map'
		pa = Path()
		pa.header = Utils.make_header(frame_id)
		pa.poses = []
		for n in points_flat:
			n2 = [n[0] * PIXELS_TO_METERS + map_x, n[1] * PIXELS_TO_METERS + map_y, 0]
			pa.poses.append(Utils.particle_to_posestamped(n2, str(frame_id)))
		path_pub.publish(pa)

	if leg_pub.get_num_connections() > 0:
		frame_id = 'map'
		pa = Path()
		pa.header = Utils.make_header(frame_id)
		pa.poses = []
		for n in points[leg]:
			n2 = [n[0] * PIXELS_TO_METERS + map_x, n[1] * PIXELS_TO_METERS + map_y, 0]
			pa.poses.append(Utils.particle_to_posestamped(n2, str(frame_id)))
		leg_pub.publish(pa)


def follower_cb(msg):
	global steer
	global reverse
	global prevmsg
	global preverr
	global off_end

	if prevmsg == None:
		prevmsg = msg
		preverr = (0,0)
		return

	pose = np.array([(msg.pose.position.x - map_x) / PIXELS_TO_METERS,
			(msg.pose.position.y - map_y) / PIXELS_TO_METERS,
			Utils.quaternion_to_angle(msg.pose.orientation)])

	posepx = [int(pose[0]), int(pose[1])]


	if dist(posepx, points[leg][-1]) < 10:
		leg += 1


	steer_mod = 0
	reverse_mod = False
	if image_processor.red_count > 20:
		steer_mod += 0.001 * image_processor.red_dist * image_processor.red_center
		if image_processor.red_dist > 0.9 and math.abs(image_processor.red_center) < 0.2:
			reverse_mod = True

	if image_processor.blue_count > 20:
		steer_mod += -0.002 * image_processor.blue_dist * image_processor.blue_center


	dist, angle = get_error(pose[0:2], pose[2], points[leg])
	angle += np.pi
	angle = angle % (np.pi*2)
	angle -= np.pi

	del_t = np.float64(msg.header.stamp.secs - prevmsg.header.stamp.secs)
	del_t += np.float64((msg.header.stamp.nsecs - prevmsg.header.stamp.nsecs)/1000000000.0)
	del_dist = (dist - preverr[0]) / del_t

	steer = dist * 0.05 + del_dist * 0.075
	
	if angle < -np.pi/2:
		steer = 0.4
	elif angle > np.pi/2:
		steer = -0.4

	reverse = allowed[posepx[1],posepx[0]] == 0
	reverse = reverse or off_end

	print dist, angle, reverse

	steer += steer_mod
	reverse = reverse or reverse_mod

	preverr = (dist, angle)


if __name__ == '__main__':
	global map_x
	global map_y
	global image_processor
	global path_pub
	global allowed
	global leg
	global prevmsg
	global preverr
	global reverse
	global off_end

	leg = 0
	reverse = False
	prevmsg = None
	preverr = None
	off_end = False

	for l in points:
		for i in range(len(l)):
			l[i] = np.array((l[i][1], l[i][0]))

	rospy.init_node("follower", anonymous=True)

	pose_sub  = rospy.Subscriber("/pf/viz/inferred_pose",
			PoseStamped, follower_cb, queue_size=1)

	pub_drive = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",
		AckermannDriveStamped, queue_size = 1)

	image_processor = ImageProcessor(None)
	image_sub = rospy.Subscriber(rospy.get_param("~image_topic", "/camera/color/image_raw"), Image, image_processor.image_cb, queue_size=1)

	allowed = np.load("/home/nvidia/catkin_ws/src/final/src/out.npy")
	for asdf in range(0,3200,50):
		print allowed[asdf,asdf]

	map_service_name = rospy.get_param("~static_map", "static_map")
	print("Getting map from service: ", map_service_name)
	rospy.wait_for_service(map_service_name)
	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map # The map, will get passed to init of sensor model
	map_x = map_msg.info.origin.position.x
	map_y = map_msg.info.origin.position.y

	path_pub = rospy.Publisher("/viz/plan", Path, queue_size = 1)
	leg_pub = rospy.Publisher("/viz/leg", Path, queue_size = 1)

	visualize()

	i = 0
	while not rospy.is_shutdown():
		i += 1
		i = i % 10
		if i == 0:
			visualize()

		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"

		if reverse:
			msg.drive.steering_angle = -steer
			msg.drive.speed = -1
		else: 
			msg.drive.steering_angle = steer
			msg.drive.speed = 1
			
		pub_drive.publish(msg)

		
