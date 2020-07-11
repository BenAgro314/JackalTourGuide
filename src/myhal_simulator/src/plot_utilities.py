#!/usr/bin/env python

import matplotlib.pyplot as plt

def plot_trajectory(traj):
    x = []
    y = []

    for pose in traj:
        x.append(pose.pose.position.x)
        y.append(pose.pose.position.y)

    plt.plot(x,y)
    

def show():
    plt.show()


if __name__ == "__main__":
    pass