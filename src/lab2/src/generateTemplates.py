#!/usr/bin/env python

import numpy as np


initial_pos = (0, 0, 0)

speed = 0.01 # (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
min_angle = -0.263370416152
max_angle = 0.31347342398
# angle: (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
increment = (max_angle-min_angle)/20

pos = initial_pos
angle = min_angle
for index in range(21):
	print "angle: ", angle
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
        print index
        print x
        print y
	angle += increment
