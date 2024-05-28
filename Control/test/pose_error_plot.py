#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 28 13:34:54 2024

@author: woolfrey
"""

import os                                                                      # For obtaining file directories
import pandas                                                                  # For reading csv files
import matplotlib.pyplot            

######################### Load the error data #################################
path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../../build/pose_error_data.csv"))

errorData = pandas.read_csv(path, header=None)

errorData.columns = ("Time",
                     "Position",
                     "Orientation",
                     "Manipulability")

###############################################################################

fig, ax = matplotlib.pyplot.subplots(3,1)

fig.suptitle("Pose Tracking Error")

labels = ["Position (mm)",
          "Orientation (deg)",
          "Manipulability"]

ax[0].plot(errorData["Time"],
           errorData["Position"]*1000, 
           color = [0,0,0])

ax[1].plot(errorData["Time"],
           errorData["Orientation"]*180/3.1416,
           color = [0,0,0])

ax[2].plot(errorData["Time"], errorData["Manipulability"], color=[0,0,0])

for i in range(0,3):
    ax[i].spines['top'].set_visible(False)
    ax[i].spines['right'].set_visible(False)
    ax[i].set_ylabel(labels[i])
    if(i < 2):
        ax[i].set_xticks([])
    else:
        ax[i].set_xlabel("Time (s)")
            