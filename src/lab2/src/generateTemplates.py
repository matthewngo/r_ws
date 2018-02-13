#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import cv2


initial_pos = (0, 0, 0)

speed = 0.5 # (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
servo_min_angle = -0.263370416152
servo_max_angle = 0.31347342398

min_angle = -0.7
max_angle = 0.7

# angle: (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
increment = (max_angle-min_angle)/40


rotation_matrix = np.array([[ -1.66533454e-16,  -1.00000000e+00,   0.00000000e+00,
         -2.60000000e-02],
       [ -4.38202247e-01,  -1.66533454e-16,  -8.98876404e-01,
          2.89000000e-01],
       [  8.98876404e-01,   0.00000000e+00,  -4.38202247e-01,
         -1.41000000e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])


K = np.array([[618.0400390625, 0.0, 321.1227722167969], [0.0, 618.6351318359375, 235.7403106689453] , [0.0, 0.0, 1.0]])

pos = initial_pos
angle = min_angle
all_temps = np.zeros((480,640), dtype=np.uint8)
for index in range(42):
	# generate template
	pos = initial_pos
	x = []
	y = []
	num = 125
	if angle > servo_max_angle or angle < servo_min_angle:
		num = 150
	for i in range(num):
		theta = pos[2]
		delta_x = (speed * np.cos(theta)) *0.01
		delta_y = (speed * np.sin(theta)) *0.01

		beta = np.arctan(0.5 * np.tan(angle))
		delta_theta = (speed / 0.33) * np.sin(2 * beta) *0.01

		pos = np.add(pos, (delta_x, delta_y, delta_theta))
		x.append(pos[0]+.1666)
		y.append(pos[1])

	x = np.array(x)
	y = np.array(y)
	z = np.zeros(x.shape[0])
	o = np.ones(x.shape[0])

	points = np.column_stack((x, y, z, o))

	transformed = np.zeros([x.shape[0], 4])

	for i, row in enumerate(points):
		transformed[i, :] = np.matmul(rotation_matrix, row)

	
	camera_frame = transformed[:,:3]

	x_div = transformed[:, 0] / transformed[:, 2]
	y_div = transformed[:, 1] / transformed[:, 2]

	camera_frame = np.column_stack((x_div, y_div, np.ones(x.shape[0])))

	pixels = np.zeros([x.shape[0], 3])

	for i, row in enumerate(camera_frame):
		pixels[i, :] = np.matmul(K, row)


	rows = pixels[:, 1].astype(np.int32)
	cols = pixels[:, 0].astype(np.int32)
	valid = (rows >= 0) & (rows < 480) & (cols >= 0) & (cols < 640)
	rows = rows[valid]
	cols = cols[valid]

	temp = np.zeros((480,640), dtype=np.uint8)

	temp[rows, cols] = 255
	all_temps[rows, cols] = 255

	# cv2.imshow("yyy", temp)
	# cv2.waitKey(0)

	# name = "good_rollouts/" + str(angle) + ".png"

	# cv2.imwrite(name, temp)

	angle += increment

# cv2.imshow("yyy", all_temps)
# cv2.waitKey(0)
cv2.imwrite("good_rollouts/all.png", all_temps)