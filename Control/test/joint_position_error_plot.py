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

# Variables
numJoints = len(data.columns)
cols = 2;
rows = round(numJoints/cols)

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
fig, ax = matplotlib.pyplot.subplots(rows, cols)

fig.suptitle("Joint Tracking Error", fontweight = 600)

jointNum = 1
for i in range(0,rows):
    for j in range(0,cols):
    
        ax[i][j].set_title("Joint " + str(jointNum), fontsize=10)              # Subfigure title

        # Plot error vs time
        ax[i][j].plot(data["Time"],
                      data["Joint " + str(jointNum)]*180/3.1416,
                      color = [0,0,0])
        
        # Removes unecessary spines
        ax[i][j].spines['top'].set_visible(False)
        ax[i][j].spines['right'].set_visible(False)
        
        # Set time label only on last row
        if(jointNum > numJoints - 3):
            ax[i][j].set_xlabel("Time (s)")                                    # Put label on last row
        else:
            ax[i][j].set_xticks([])
            ax[i][j].spines['bottom'].set_visible(False)
        
        # Put y-axis label only on 1st column
        if(j == 0):
            ax[i][j].set_ylabel('Deg')
            
        jointNum = jointNum + 1                                                # Increment the joint number