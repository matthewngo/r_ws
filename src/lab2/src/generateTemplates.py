#!/usr/bin/env python

import numpy as np


initial_pos = (0, 0, 0)

speed = 0.01 # (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
min_angle = -0.263370416152
max_angle = 0.31347342398
# angle: (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
increment = (max_angle-min_angle)/20

rotation_matrix = np.array([[ -1.60812265e-16,  -3.74606190e-01,   9.27184018e-01, 2.54000000e-01],
			   [ -1.00000000e+00,   6.02412698e-17,  -1.49102562e-16, -2.60000000e-02],
			   [  0.00000000e+00,  -9.27184018e-01,  -3.74606190e-01, 1.98000000e-01],
			   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])

K = np.array([[618.0400390625, 0.0, 321.1227722167969], [0.0, 618.6351318359375, 235.7403106689453] , [0.0, 0.0, 1.0]])

pos = initial_pos
angle = min_angle
for index in range(21):
	pos = initial_pos
	x = []
	y = []
	for i in range(150):
		theta = pos[2]
		delta_x = (speed * np.cos(theta)) 
		delta_y = (speed * np.sin(theta))

		beta = np.arctan(0.5 * np.tan(angle))
		delta_theta = (speed / 0.33) * np.sin(2 * beta)

		pos = np.add(pos, (delta_x, delta_y, delta_theta))
			#print (pos[0], pos[1])
		x.append(pos[0])
		y.append(pos[1])
	print "angle: ", angle
	print index
	#print x
	#print y

	x = np.array(x)
	y = np.array(y)
	z = np.zeros(x.shape[0])
	o = np.ones(x.shape[0])

	points = np.column_stack((x, y, z, o))

	transformed = np.zeros([x.shape[0], 4])

	for i, row in enumerate(points):
		transformed[i, :] = np.matmul(rotation_matrix, row)

	x_div = transformed[:, 0] / transformed[:, 2]
	y_div = transformed[:, 1] / transformed[:, 2]

	camera_frame = np.column_stack((x_div, y_div, transformed[:, 2]))

	pixels = np.zeros([x.shape[0], 3])

	for i, row in enumerate(camera_frame):
		pixels[i, :] = np.matmul(K, row)

	for i, row in enumerate(pixels):
		pixels[i,:] = pixels[i,:] / pixels[i, 2]

	print pixels

	
	angle += increment
