import rospy
import numpy as np

class ReSampler:
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles
    self.weights = weights
    self.particle_indices = None  
    self.step_array = None
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  def resample_naiive(self):
    self.state_lock.acquire()
    # Use np.random.choice to re-sample 
    # YOUR CODE HERE
    indices = range(self.particles.shape[0])
    picked_indices = np.random.choice(indices, len(self.particles), True, self.weights);
    self.particles[:] = np.take(self.particles, picked_indices)
    
    self.state_lock.release()
  
  def resample_low_variance(self):
    self.state_lock.acquire()
    # Implement low variance re-sampling
    # YOUR CODE HERE
    new_particles = []
    num_particles = len(self.particles)
    rand_num = np.random.random_sample()/num_particles
    curr_weight = self.weights[0]
    index = 0
    for particle in range(num_particles):
      u = rand_num + (particle-1)/num_particles
      while (u < curr_weight):
        index = index + 1
        curr_weight = curr_weight + self.weight[index]
        new_particles.append(self.particles[index])  
    self.particles[:] = new_particles[:]

    self.state_lock.release()

