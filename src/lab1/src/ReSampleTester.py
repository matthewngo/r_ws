import matplotlib.pyplot as plt
import rospy
import numpy as np
import ReSample as rs

particle_ranges = [100, 200, 500, 1000, 4000]
naiive_variances = []
low_var_variances = []
for particle_range in particle_ranges:
  print particle_range

  particles = np.random.rand(particle_range)
  # particles = np.stack([np.copy(particles), np.copy(particles), np.copy(particles)], axis = 1)

  weights = np.ones(particle_range)
  weights = weights / np.sum(weights)
  print np.sum(weights)
  
  naiive_particles = np.copy(particles)
  rsamp1 = rs.ReSampler(naiive_particles, np.copy(weights))
  rsamp1.resample_naiive()
  naiive_variances.append(naiive_particles.var())
  
  low_var_particles = np.copy(particles)
  rsamp2 = rs.ReSampler(low_var_particles, np.copy(weights))
  rsamp2.resample_low_variance()
  low_var_variances.append(low_var_particles.var())

print naiive_variances
print low_var_variances

plt.plot(particle_ranges, naiive_variances, 'r', particle_ranges, low_var_variances, 'b')
plt.show()
