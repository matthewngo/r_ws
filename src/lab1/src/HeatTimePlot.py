import matplotlib.pyplot as plt
import rospy
import numpy as np

particle_steps = [1, 2, 3, 5, 10, 20, 30]
step_times = [139.84, 54.26, 36.17, 26.75, 22.59, 21.33, 20.97]

plt.plot(particle_steps, step_times, 'bo')
plt.xlabel("Distance Between Generated Particles (pixels)")
plt.ylabel("Processing Time (seconds)")
plt.show()
