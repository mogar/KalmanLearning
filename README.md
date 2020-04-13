# Kalman Filter Learning

I've been learning about Kalman Filters. This project is just an implementation that I did to cement my understanding.

For more about Kalman Filters: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

That book goes over the use of the filterpy library. I ended up implementing a subset of the Kalman Filter features from that library myself to verify that I was understanding things correctly.

## Files

* kf.py: Kalman Filter implementation for arbitrary linear problems (you have to derive the KF matrices yourself)
* target.py: a simple class that implements an object moving on the shell of a sphere. Used to test KF tracking.
* kf_display.py: plotting helper class for target.py related experiments
* tracker.py: a KF that tracks the target and gives you a best estimate of where the target is. Assumes arbitrary 3D motion, not constrained to a sphere as the target actually is. Also lets you play with some parameters to see how they impact the convergence and accuracy of the filter.

### ToDo

* interceptor.py: A simple robot able to move in 3D (quadcopter) that is intended to intercept the target (demonstrates use of control variables in a KF)

## Usage

You'll need python, numpy, and matplotlib.

To display a simple movie of the tracker, use:

    python3 tracker.py

To display a simple movie of the interceptor, use:

    not yet implemented

These files have a set of variables near the top that you can tweak to experiment.