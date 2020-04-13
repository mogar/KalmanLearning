
"""
Kalman Filter
This is my attempt at a generalized Kalman Filter. The class tracks the
KF variables and performs innovation. All of the variables themselves
need to be created and passed to the KF object when it's created.

Based on the KFs developed in rlabbe's Kalman Filter book:
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
"""

import numpy as np

class KalmanFilter():
    def __init__(self,
                 x = 0,  # default 1 variable state
                 P = 10, # default variance of 10 (kinda meaningless without units)
                 F = 1,  # default to assuming that nothing changes in our state
                 Q = 10, # default process has same noise as default state 
                 B = 0,  # default to no control input
                 H = 1,  # default measurement is just the state value
                 R = 0   # default measurement noise is 0 (perfect measurements)
                 ):
        """ Simple Kalman Filter Implementation.
        """
        self.x = np.array(x) # Kalman Filter state mean   (best estimate of what we're tracking)
        self.P = np.array(P) # state covariance           (how accurate we think our state is)
        self.F = np.array(F) # process model              (how an undisturbed state changes over time)
        self.Q = np.array(Q) # process model covariance   (how much we believe our own process model)
        self.B = np.array(B) # control model              (how disturbing a state causes it to change, only used if we can control our target in some way)
        self.H = np.array(H) # measurement function       (converts a state into what measurement we expect)
        self.R = np.array(R) # sensor noise covariance (we're assuming this is fixed)
        
        # TODO: error checking to verify that inputs have correct array sizing



    def innovate(self, z, u=0):
        """ Kalman Filter innovation is just a single step of prediction and update
        """
        self.predict(u)
        self.update(z)

    def predict(self, u=0):
        """ Compute the prior for this innovation
        """
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T)+self.Q
        

    def update(self, z):
        """ Compute the posterior for this innovation
        """
        # calculate residual      (difference between prediction and measurement)
        y = z - np.dot(self.H, self.x)
        # calculate Kalman gain   (how much we believe in measurement vs prediction)
        PHt = np.dot(self.P, self.H.T)
        S = np.dot(self.H, PHt) + self.R
        K = np.dot(PHt, np.linalg.inv(S))
        # generate new state
        self.x = self.x+np.dot(K, y)
        # generate new state belief
        # this is defined as P = (I-KH)P_predict, but this is an unstable thing to calculate
        # so instead we do: P = (I-KH)P(I-KH)' + KRK'
        I_KH = np.eye(self.x.size) - np.dot(K, self.H)
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(K, self.R), K.T)

    
    def get_state(self):
        """ Return the current state of the filter
        """
        return self.x