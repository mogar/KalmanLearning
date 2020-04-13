#!/usr/bin/env python3

"""
A kalman filter based tracker.
Attempts to track the location of a target. Plots the
predicted location and the actual location of the target.

"""

import numpy as np

import target as t
import kf
import kf_display as kfd

###############################################################################
# configuration stuff

dt = 0.1
radius = 10

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

def measure_target(target_pos, measurement_noise):
    dim = len(target_pos)
    m = np.zeros(dim)
    for i in range(dim):
        m[i] = target_pos[i] + np.random.normal(0, measurement_noise)
    return m

###############################################################################
# set up tracking filter
# note that we're assuming the target is free to move in 3D, which is not actually true

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

tracker = kf.KalmanFilter(x = x, P=P, F=F, Q=Q, H=H)

###############################################################################
# run and plot

for i in range(timesteps):
    # update target position and measurement
    kf_target.update()
    target_pos = kf_target.get_pos_shell()
    target_meas = measure_target(target_pos, sensor_noise)

    # use measurement to update filter
    tracker.innovate(z=target_meas)
    tracker_pos = tracker.get_state()

    # draw it
    kf_plots.display(target_pos, tracker_pos)
    kf_plots.pause(dt)

