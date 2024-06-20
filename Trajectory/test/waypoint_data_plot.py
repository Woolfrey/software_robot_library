#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 17 07:58:27 2024

@author: woolfrey
"""
import os                                        # For obtaining file directories
import pandas                                    # For reading csv files
import matplotlib.pyplot                         # For plotting data

########## Load the data from file ##############
path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../../build/trajectory_test_data.csv"))

data = pandas.read_csv(path,header=None)

data.columns = ["Time", "Position", "Velocity", "Acceleration"] # Give a name to each column

path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../../build/waypoint_test_data.csv"))

waypoints = pandas.read_csv(path,header=None)

waypoints.columns = ["Time", "Position"]

################### Plot the data ###############
fig,ax = matplotlib.pyplot.subplots(3,1, dpi = 300)         # Create figure with 3 subplots

ax[0].plot(data["Time"],data["Position"],     color=[0,0,0])
ax[1].plot(data["Time"],data["Velocity"],     color=[0,0,0])
ax[2].plot(data["Time"],data["Acceleration"], color=[0,0,0])
ax[2].set_xlabel("Time")

ax[0].scatter(waypoints["Time"], waypoints["Position"], color = [0, 0, 0])     # Add markers for the waypoints

##### Clean up the figure and remove spines #####
labels = ["Position", "Velocity", "Acceleration"]
for i in range(0,3):
    ax[i].spines['top'].set_visible(False)
    ax[i].spines['right'].set_visible(False)
    ax[i].set_ylabel(labels[i])
    ax[i].get_yaxis().set_ticks([data[labels[i]].min(), data[labels[i]].max()])
    if i < 2:
        ax[i].spines['bottom'].set_visible(False)
        ax[i].get_xaxis().set_ticks([])
    if i > 0:
        ax[i].axhline(y=0, color=[0.2,0.2,0.2], linewidth=0.5)