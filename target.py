
"""
The Kalman Filter Target is constrained to a spherical shell. It moves along this spherical shell
according to the phi and theta velocities and accelerations. Velocity and position updates are 
deterministic, but acceleration has a Gaussian noise added to it with mean 0 and variance as 
initialized.

There's really nothing Kalman Filter specific here, it's just a target to try and track with an
actual Kalman Filter.

Internally, the position of the target is represented using spherical coordinates. The radius
of the sphere is fixed during time updates. For more on the coordinates, see:
https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.7%3A_Cylindrical_and_Spherical_Coordinates
"""

import numpy as np

class KFTarget():
    def __init__(self,
                 start_radius = 10.0, 
                 start_theta = 0.0, 
                 start_phi = 0.0, 
                 start_vel_theta = 10.0, 
                 start_vel_phi = 5.0,
                 accel_theta = 1.0,
                 accel_phi = 2.0,
                 accel_noise = 1.0,
                 dt = 0.1):
        """ initialize the target
        angles are all in degrees
        """
        self.pos_radius = start_radius
        self.pos_theta = start_theta
        self.pos_phi = start_phi
        self.vel_theta = start_vel_theta
        self.vel_phi = start_vel_phi
        self.accel_theta = accel_theta
        self.accel_phi = accel_phi
        self.accel_noise = accel_noise
        self.dt = dt

    def update(self):
        """ update the location of the target
        """
        # update location
        self.pos_theta = self.pos_theta + self.vel_theta*self.dt + 0.5*self.accel_theta*self.dt**2
        self.pos_phi = self.pos_phi + self.vel_phi*self.dt + 0.5*self.accel_phi*self.dt**2
        
        # update velocities
        self.vel_theta = self.vel_theta + self.accel_theta*self.dt
        self.vel_phi = self.vel_phi + self.accel_phi*self.dt

        # update acceleration based on noise
        # normal distribution with mean = 0, var = noise
        self.accel_theta = self.accel_theta + np.random.normal(0, self.accel_noise)
        self.accel_phi = self.accel_phi + np.random.normal(0, self.accel_noise)

    def get_pos_shell(self):
        """ get the cartesian position of the target
        """
        x = self.pos_radius*np.sin(np.pi/180.0*self.pos_phi)*np.cos(np.pi/180.0*self.pos_theta)
        y = self.pos_radius*np.sin(np.pi/180.0*self.pos_phi)*np.sin(np.pi/180.0*self.pos_theta)
        z = self.pos_radius*np.cos(np.pi/180.0*self.pos_phi)
        return (x, y, z)