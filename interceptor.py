#!/usr/bin/env python3

"""
A kalman filter based interceptor.
Simulates a robot attempting to intercept a target.
Plots the locations of both target and interceptor.

"""

import numpy as np

import target as t
import kf
import kf_display as kfd

###############################################################################
# configuration stuff

dt = 0.1 # timestep
radius = 10 # radius of shell that target is constrained to

runtime = 30 # runtime in seconds
timesteps = round(runtime/dt)
print("runtime: {0}, timesteps: {1}".format(runtime, timesteps))

###############################################################################
# set up plotting stuff
kf_plots = kfd.KFDisplay(1.5*radius)

###############################################################################
# set up target
kf_target = t.KFTarget(dt=dt, start_radius=radius)

###############################################################################
# measurement function

sensor_noise = 2

# TODO: update for relative measurements
def measure_target(target_pos, interceptor_pos, measurement_noise):
    dim = len(target_pos)
    m = np.zeros(dim)
    for i in range(dim):
        m[i] = target_pos[i] + np.random.normal(0, measurement_noise)
    return m


###############################################################################
# interceptor functions

max_acceleration = 2 # degrees/s^2

interceptor_loc = (0, 0, 0)
interceptor_vel = (0, 0, 0)

def interceptor_update(loc, vel, control, time):
    loc = loc + (vel*time)
    vel = vel + control
    return (loc, vel)

def generate_control(target_pos):
    # this is where you could implement your intelligent controller with
    # e.g. PID or state-space optimal controller or whatever
    # For this example, we just do something pretty naive
    return (0,0,0) #just do nothing for now


###############################################################################
# set up interceptor filter
# note that we're assuming the target is free to move in 3D, which is not actually true
# this is pretty similar to the tracking filter, but with an additional control input and
# related matrices.
#
# The state is also different. In the tracking example, the state is position and velocity
# in a global coordinate system. In this example, we are tracking the target relative to our
# interceptor robot, and the control input is intended to drive the difference to 0.
#
# There are lots of other ways to do this, e.g. using a KF for the target and another for
# the interceptor and then using both of their states to calculate the control input. I
# picked this method kind of at random.

# state and state covariance
#    x, y, z, vx, vy, vz
x = [0, 0, 0, 0, 0, 0] # 6D filter state, 3 for position and 3 for velocity (could improve by also tracking acceleration)
P = np.eye(6)*radius**2 # assuming no correlations between state variables, stdev of 10 for each var

# process model and process covariance
# pos = pos+dt*vel, vel = vel
# assuming no acceleration
F = [[1, 0, 0, dt, 0, 0],
     [0, 1, 0, 0, dt, 0],
     [0, 0, 1, 0, 0, dt],
     [0, 0, 0, 1, 0, 0],
     [0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 1]]
Q = np.eye(6)*radius**2

# measurement model
# assumes we are just measuring position directly
H = [[1, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0]]

# Control model
# assumes that our control is just an acceleration
B = [[0, 0, 0, 1, 0, 0],
     [0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 1]]

tracker = kf.KalmanFilter(x = x, P=P, F=F, Q=Q, H=H, B=B)

###############################################################################
# run and plot

u = (0, 0, 0)
for i in range(timesteps):
    # update target position and measurement
    kf_target.update()
    target_pos = kf_target.get_pos_shell()
    target_meas = measure_target(target_pos, interceptor_loc, sensor_noise)

    # use measurement to update filter
    tracker.innovate(z=target_meas, u=u)
    tracker_pos = tracker.get_state()

    # generate interceptor control based on tracker state
    u = generate_control(target_pos)
    (interceptor_loc, interceptor_vel) = interceptor_update(interceptor_loc, interceptor_vel, u, dt)

    # draw it
    kf_plots.display(target_pos, tracker_pos)
    kf_plots.pause(dt)

