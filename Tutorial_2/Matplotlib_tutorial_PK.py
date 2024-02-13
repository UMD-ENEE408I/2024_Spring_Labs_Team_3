# Matplotlib tutorial
# Peter Krepkiy
# ENEE408I lab 2

# Problem (1)

import numpy as np
import matplotlib.pyplot as plt 



x = np.linspace(0, 2*np.pi, 360)

# Calculate y values (sine function)
y = np.sin(x)

fig, ax = plt.subplots()

# Create the plot
plt.plot(x, y)

# Problem (2)

ax.set_xlabel('TIME (S)')
ax.set_ylabel('SINE WAVE AMPLITUDE')

plt.show()

# Problem 3

from mpl_toolkits.mplot3d import Axes3D

x = np.linspace(-10, 10, 100)
y = np.linspace(-10, 10, 100)
x, y = np.meshgrid(x, y)


z = np.sin(np.sqrt(x**2 + y**2))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Surface plot
surf = ax.plot_surface(x, y, z, cmap='viridis')

plt.show()


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


