#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import math

points = [(609, 593), (1788, 2010), (2000, 1978)]

PIXELS_TO_METERS = 0.02

curr = points[0]
nxt = points[1]

steer = 0


def follower_cb(msg):
	global steer
	last_pose = [msg.pose.position.x / PIXELS_TO_METERS,
								msg.pose.position.y / PIXELS_TO_METERS]
	
	dist = ((nxt[1] - curr[1]) * last_pose[0] - (nxt[0] - curr[0]) * last_pose[1] + nxt[0]*curr[1] - nxt[1]*curr[0]) / math.sqrt(((nxt[1] - curr[1])**2) + ((nxt[0] - curr[0]**2)))

	if dist > 0:
		print '-1', dist
		steer = -1
	else:
		print '1', dist
		steer = 1


if __name__ == '__main__':
	rospy.init_node("follower", anonymous=True)

	pose_sub  = rospy.Subscriber("/pf/viz/inferred_pose",
			PoseStamped, follower_cb, queue_size=1)

	pub_drive = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",
		AckermannDriveStamped, queue_size = 1)

	

	while not rospy.is_shutdown():
		#drive forward with constant speed and lf.angle


		msg = AckermannDriveStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "base_link"

		msg.drive.steering_angle = steer
		msg.drive.speed = 1
			
		pub_drive.publish(msg)