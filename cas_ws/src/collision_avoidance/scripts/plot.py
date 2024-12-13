#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Initialize lists to store the positions
drone1_x, drone1_y, drone1_z = [], [], []
drone2_x, drone2_y, drone2_z = [], [], []

# Read data from the text file
with open('/home/usr/cas_ws/src/collision_avoidance/src/trajectories.txt', 'r') as file:
    # Skip the header line
    next(file)
    
    for line in file:
        data = line.strip().split()
        if len(data) == 6:  # Ensure we have 6 values
            x1, y1, z1, x2, y2, z2 = map(float, data)
            drone1_x.append(x1)
            drone1_y.append(y1)
            drone1_z.append(z1)
            drone2_x.append(x2)
            drone2_y.append(y2)
            drone2_z.append(z2)

# Plotting the trajectories
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot Drone 1 trajectory
ax.plot(drone1_x, drone1_y, drone1_z, color='b', label='Drone 1 Trajectory')
# Plot Drone 2 trajectory
ax.plot(drone2_x, drone2_y, drone2_z, color='r', label='Drone 2 Trajectory')

# Setting labels
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Trajectories of Drones')
ax.legend()

# Set z-axis limits
ax.set_zlim(65, 75)

# Show the plot
plt.show()
