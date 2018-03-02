import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt

import time
import sys
import rospy
import rosbag
import numpy as np
import scipy.signal
import utils as Utils

import torch
import torch.nn as nn
import torch.utils.data
from torch.autograd import Variable

INPUT_SIZE=8
OUTPUT_SIZE=3

def make_input(v, delta, dt):
    ret = torch.zeros(INPUT_SIZE).cuda()
    ret[3] = np.cos(0)
    ret[4] = np.sin(0)
    ret[5] = v
    ret[6] = delta
    ret[7] = dt
    return ret

def rollout(m, nn_input, N):
    pose = torch.zeros(3).cuda()
    #print(pose.cpu().numpy())
    print str(pose[0]) + "\t" + str(pose[1])
    for i in range(N):
        out = m(Variable(nn_input))
        pose.add_(out.data)
        # Wrap pi
        if pose[2] > 3.14:
            pose[2] -= 3.14
        if pose[2] < -3.14:
            pose[2] += 3.14
        nn_input[0] = out.data[0]
        nn_input[1] = out.data[1]
        nn_input[2] = out.data[2]
        #nn_input[3] = pose[2]
        nn_input[3] = np.cos(pose[2])
        nn_input[4] = np.sin(pose[2])
        #print(pose.cpu().numpy())
        print str(pose[0]) + "\t" + str(pose[1])

def test_model(m, N, dt = 0.1):
    s = INPUT_SIZE 
    print("Nothing")
    nn_input = make_input(0, 0, dt)
    rollout(m, nn_input, N)
    print("")

    print("Forward")
    nn_input = make_input(0.7, 0, dt)
    rollout(m, nn_input, N)
    print("")

    print("Backward")
    nn_input = make_input(-0.7, 0, dt)
    rollout(m, nn_input, N)
    print("")

    print("Left")
    nn_input = make_input(0.7, -0.25, dt)
    rollout(m, nn_input, N)
    print("")

    print("Right")
    nn_input = make_input(0.7, 0.3, dt)
    rollout(m, nn_input, N)
    print("")

#doTraining(model, "test.out", opt, 5000)
model = torch.load("/home/nvidia/catkin_ws/src/lab3/src/test.out")
test_model(model, 10)

# Reporting validation set error
xerr, yerr, therr = 0,0,0
valsize = x_val.shape[0]
for i in range(valsize):
	y_pred = model(Variable(x_val[i]))
	xerr += abs(y_val[i][0] - y_pred.data[0])
	yerr += abs(y_val[i][1] - y_pred.data[1])
	therr += abs(y_val[i][2] - y_pred.data[2])
print xerr / valsize
print yerr / valsize
print therr / valsize