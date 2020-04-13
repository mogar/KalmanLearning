
"""
Plotting class to help with KF experiments on the target.py example.
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class KFDisplay():
    def __init__(self, lim):
        # TODO: may want to make a shell in the plot to make it easier to see stuff
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion() # turn on interactive mode

        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_zlim(-lim, lim)
        self.target_marks = ('r', '^')
        self.tracker_marks = ('b', 'o')

    def display(self, target_pos, tracker_pos):
        for artist in plt.gca().lines + plt.gca().collections:
            artist.remove()
        self.ax.scatter(target_pos[0], target_pos[1], target_pos[2], c=self.target_marks[0], marker=self.target_marks[1])
        self.ax.scatter(tracker_pos[0], tracker_pos[1], tracker_pos[2], c=self.tracker_marks[0], marker=self.tracker_marks[1])
        plt.show()

    def pause(self, seconds):
        plt.pause(seconds)