import serial
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from math import atan2, asin, degrees

import matplotlib.pyplot as plt

# Connect to Arduino serial port
ser = serial.Serial('COM14', 9600)  # Replace 'COM3' with your Arduino's serial port

# Initialize figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialize vectors for x, y, z axes
x_vec = np.array([1, 0, 0])
y_vec = np.array([0, 1, 0])
z_vec = np.array([0, 0, 1])

# Function to convert quaternion to Euler angles
def quaternion_to_euler(q):
    roll = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1]**2 + q[2]**2))
    pitch = asin(2 * (q[0] * q[2] - q[3] * q[1]))
    yaw = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
    return degrees(roll), degrees(pitch), degrees(yaw)

# Main loop to read quaternion data from Arduino and update the graph
while True:
    # Read quaternion data from Arduino
    quaternion_str = ser.readline().decode().strip()
    quaternion = [float(x) for x in quaternion_str.split(',')]

    # Convert quaternion to Euler angles
    roll, pitch, yaw = quaternion_to_euler(quaternion)

    # Clear previous plot
    ax.cla()

    # Plot x, y, z vectors
    ax.quiver(0, 0, 0, x_vec[0], x_vec[1], x_vec[2], color='r', label='X')
    ax.quiver(0, 0, 0, y_vec[0], y_vec[1], y_vec[2], color='g', label='Y')
    ax.quiver(0, 0, 0, z_vec[0], z_vec[1], z_vec[2], color='b', label='Z')

    # Plot rotated vectors based on Euler angles
    ax.quiver(0, 0, 0, *np.dot(x_vec, roll), color='r', linestyle='dashed')
    ax.quiver(0, 0, 0, *np.dot(y_vec, pitch), color='g', linestyle='dashed')
    ax.quiver(0, 0, 0, *np.dot(z_vec, yaw), color='b', linestyle='dashed')

    # Set plot limits and labels
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set title with Euler angles
    ax.set_title(f'Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°')

    # Show the plot
    plt.pause(0.01)

# Close the serial connection
ser.close()