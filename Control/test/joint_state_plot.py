#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 22 11:20:59 2024

@author: woolfrey
"""

import os                                                                      # For obtaining file directories
import pandas                                                                  # For reading csv files
import matplotlib.pyplot                                                       # For plotting data


######################### Load the joint limits ###############################
path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../../build/joint_limits.csv"))

jointLimits = pandas.read_csv(path, header=None)

jointLimits.columns = ["Lower", "Upper", "Velocity"]


######################### Load the position data ##############################
path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../../build/joint_position_data.csv"))

positionData = pandas.read_csv(path, header=None)

positionData.columns = ["Time",
                        "Joint 1",
                        "Joint 2",
                        "Joint 3",
                        "Joint 4",
                        "Joint 5",
                        "Joint 6",
                        "Joint 7",
                        "Joint 8"]

# Normalise the position values between -1 and 1
for i in range(0,len(positionData.columns)-1):
    minimum = jointLimits["Lower"][i]
    maximum = jointLimits["Upper"][i]
    raange = maximum - minimum
    
    for j in range(0,len(positionData)):
            pos = positionData["Joint " + str(i+1)][j]
            positionData["Joint " + str(i+1)][j] = 2*(pos-minimum)/raange - 1
    

# Plot the data
fig, ax = matplotlib.pyplot.subplots()                                         # Create figure object

# Add reference lines to aid visual discrimination
ax.axhline(y =  1.0, color = [0.8, 0.8, 0.8], linestyle = '-') 
ax.axhline(y = -1.0, color = [0.8, 0.8, 0.8], linestyle = '-') 

# Plot each joint position vs. time
for i in range(0,len(positionData.columns)-1):
      ax.plot(positionData["Time"],                                            # x-axis data
              positionData["Joint " + str(i+1)],                               # y-axis data
              color = [0,0,0])
    
# Format the figure
fig.suptitle("Normalised Joint Positions")                                     # Give it a title
ax.spines['top'].set_visible(False)                                            # Remove top line
ax.spines['right'].set_visible(False)                                          # Remove right line
ax.spines['bottom'].set_position(('data', -1))                                 # Set the intercept of the x and y axes
ax.set_xlabel("Time (s)")                                                      # Label the bottom axis
ax.set_yticks([-1,0,1])                                                        

matplotlib.pyplot.show()

########################### Load the velocity data ############################

path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "../../build/joint_velocity_data.csv"))

velocityData = pandas.read_csv(path, header=None)

velocityData.columns = ["Time",
                        "Joint 1",
                        "Joint 2",
                        "Joint 3",
                        "Joint 4",
                        "Joint 5",
                        "Joint 6",
                        "Joint 7",
                        "Joint 8"]

# Normalise the position values between -1 and 1
for i in range(0,len(velocityData.columns)-1):
    limit = jointLimits["Velocity"][i]
    
    for j in range(0,len(positionData)):
            pos = velocityData["Joint " + str(i+1)][j]
            positionData["Joint " + str(i+1)][j] = (pos-limit)/limit + 1
    

# Plot the data
fig, ax = matplotlib.pyplot.subplots()                                         # Create figure object

# Add reference lines to aid visual discrimination
ax.axhline(y =  1.0, color = [0.8, 0.8, 0.8], linestyle = '-') 

# Plot each joint position vs. time
for i in range(0,len(positionData.columns)-1):
      ax.plot(positionData["Time"],                                            # x-axis data
              positionData["Joint " + str(i+1)],                               # y-axis data
              color = [0,0,0])
    
# Format the figure
fig.suptitle("Normalised Joint Velocities")                                    # Give it a title
ax.spines['top'].set_visible(False)                                            # Remove top line
ax.spines['right'].set_visible(False)                                          # Remove right line
ax.spines['bottom'].set_position(('data', -1))                                 # Set the intercept of the x and y axes
ax.set_xlabel("Time (s)")                                                      # Label the bottom axis
ax.set_yticks([-1,0,1])     