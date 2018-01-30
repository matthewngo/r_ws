import rosbag
from MotionModel import OdometryMotionModel, KinematicMotionModel
import numpy as np
import matplotlib.pyplot as mpl
import rospy

bag = rosbag.Bag('../bags/loop.bag')

MAX_PARTICLES = 500

particles = np.zeros((MAX_PARTICLES,3)) # Numpy matrix of dimension MAX_PARTICLES x 3
weights = np.ones(MAX_PARTICLES) / float(MAX_PARTICLES) # Numpy matrix containig weight for each particle

#mm = OdometryMotionModel(particles, None)
mm = KinematicMotionModel(particles, None)

inferred_x = []
inferred_y = []
real_x = []
real_y = []
final_x = []
final_y = []

for topic, msg, t in bag.read_messages(topics=['/pf/ta/viz/inferred_pose']):
    if t.to_nsec() < 1515475100000000000: #1515474470000000000
        continue
    for i in range(MAX_PARTICLES):
        particles[i][0] = msg.pose.position.x
        particles[i][1] = msg.pose.position.y
        particles[i][2] = msg.pose.orientation.y
    break

n = 0
for topic, msg, t in bag.read_messages(topics=['/vesc/sensors/core','/pf/ta/viz/inferred_pose','/vesc/sensors/servo_position_command']): # '/vesc/odom' '/vesc/sensors/servo_position_command'
    #print msg
    #print t.to_nsec()
    if t.to_nsec() < 1515475100000000000: #1515475100000000000
        continue
    n+=1
    if n > 60:
        break
    #print topic
    if topic == '/vesc/sensors/core': #/vesc/odom
        mm.motion_cb(msg)
        for i in range(MAX_PARTICLES):
            inferred_x.append(particles[i][0])
            inferred_y.append(particles[i][1])
    elif topic == '/vesc/sensors/servo_position_command':
        mm.servo_cb(msg)
    else:
        real_x.append(msg.pose.position.x)
        real_y.append(msg.pose.position.y)
        #print(msg)
    #print(particles)
    #inferred_x
bag.close()

print real_x

for i in range(MAX_PARTICLES):
    final_x.append(particles[i][0])
    final_y.append(particles[i][1])

colors = []
for i in range(len(final_x)):
    colors.append([1,0,0])
for i in range(len(real_x)):
    colors.append([0,1,0])

#final_x.extend(real_x)
#final_y.extend(real_y)

#mpl.scatter(inferred_x, inferred_y, c=colors, alpha=0.05)
mpl.scatter(final_x, final_y, c=[1,0,0], alpha=0.1)
mpl.show()
