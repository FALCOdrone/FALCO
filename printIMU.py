import matplotlib.pyplot as plt

# Read the data from imu.log
with open('imu.log', 'r') as file:
    data = file.readlines()

# Extract speed, and acceleration values
position = {'x': [], 'y': [], 'z': []}
speed = {'x': [], 'y': [], 'z': []}
acceleration = {'x': [], 'y': [], 'z': []}

for line in data:
    values = line.strip().split(',')
    position['x'].append(float(values[0]))
    position['y'].append(float(values[1]))
    position['z'].append(float(values[2]))
    speed['x'].append(float(values[3]))
    speed['y'].append(float(values[4]))
    speed['z'].append(float(values[5]))
    acceleration['x'].append(float(values[6]))
    acceleration['y'].append(float(values[7]))
    acceleration['z'].append(float(values[8]))

# Create time axis
time = [i*0.1 for i in range(len(position['x']))]

# Plot x axis
plt.subplot(3, 1, 1)
plt.plot(time, position['x'], label='p')
plt.plot(time, speed['x'], label='v')
plt.plot(time, acceleration['x'], label='a')
plt.xlabel('Time')
plt.title('X Axis')

# Plot y axis
plt.subplot(3, 1, 2)
plt.plot(time, position['y'], label='p')
plt.plot(time, speed['y'], label='v')
plt.plot(time, acceleration['y'], label='a')
plt.xlabel('Time')
plt.title('Y Axis')

# Plot z axis
plt.subplot(3, 1, 3)
plt.plot(time, position['z'], label='p')
plt.plot(time, speed['z'], label='v')
plt.plot(time, acceleration['z'], label='a')
plt.xlabel('Time')
plt.title('Z Axis')

# Adjust layout and display the plot
plt.tight_layout()
plt.show()