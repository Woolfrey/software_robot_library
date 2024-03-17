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
                                    "../../build/polynomial_test_data.csv"))

data = pandas.read_csv(path)                     # Read the file

data.columns = ["Time", "Position", "Velocity", "Acceleration"] # Give a name to each column

################### Plot the data ###############
fig,ax = matplotlib.pyplot.subplots(3,1)         # Create figure with 3 subplots

fig.suptitle("5th Order Polynomial") # <-- MAKE SURE THIS IS CORRECT

ax[0].plot(data["Time"],data["Position"],     color=[0,0,0])
ax[1].plot(data["Time"],data["Velocity"],     color=[0,0,0])
ax[2].plot(data["Time"],data["Acceleration"], color=[0,0,0])
ax[2].set_xlabel("Time")

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