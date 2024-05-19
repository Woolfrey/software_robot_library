#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 19 15:59:03 2024

@author: woolfrey
"""

import os                                                                      # For obtaining file directories
import pandas                                                                  # For reading csv files
import matplotlib.pyplot                                                       # For plotting data

########## Load the data from file ##############
path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../../build/joint_position_error_data.csv"))

data = pandas.read_csv(path)                                                   # Read the file

# NOTE TO SELF: I need to automate this for any number of joints...
data.columns = ["Time",
                "Joint 1",
                "Joint 2",
                "Joint 3",
                "Joint 4",
                "Joint 5",
                "Joint 6",
                "Joint 7",
                "Joint 8"]

################### Plot the data ###############
fig, ax = matplotlib.pyplot.subplots(4,2)

fig.suptitle("Joint Tracking Error")

ax[0][0].plot(data["Time"], data["Joint 1"], color = [0,0,0])

# for i in range(0,7):
#     ax[i].plot(data["Time"],data["Joint " + str(i+1)], color = [0,0,0])

# ax[7].set_xlabel("Time")                    


##### Clean up the figure and remove spines #####
# labels = ["Position", "Velocity", "Acceleration"]
# for i in range(0,8):
#     ax[i].spines['top'].set_visible(False)
#     ax[i].spines['right'].set_visible(False)
#     ax[i].set_ylabel(labels[i])
#     ax[i].get_yaxis().set_ticks([data[labels[i]].min(), data[labels[i]].max()])
#     if i < 4:
#         ax[i].spines['bottom'].set_visible(False)
#         ax[i].get_xaxis().set_ticks([])
#     if i > 0:
#         ax[i].axhline(y=0, color=[0.2,0.2,0.2], linewidth=0.5)