# SciPy tutorial
# Peter Krepkiy
# ENEE408I lab 2

# Problem (1)

import numpy as np
from scipy.linalg import solve

# coefficient A matrix and solution b matrix
A = np.array([[3, 1],[1, 2]])

b = np.array([9, 8])

solution = solve(A, b)

print("Solution:")
print("x =", solution[0])
print("y =", solution[1])

# Problem (2)

from scipy.optimize import minimize 

def y(x):
    return x**2 + 2*x

result = minimize(y, 0)

# Extract the minimum value
minimum_value = result.fun

# Extract the value of x at the minimum
minimum_x = result.x[0]

print("Minimum value:", result.fun)

# Problem (3)

import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

x = np.linspace(0, 1, 1000)

# Calculate the function values
y = np.sin(100*np.pi*x) + 0.5*np.sin(160*np.pi*x)

# Perform the Fourier transform
fft_y = fft(y)

# Get frequencies
freqs = fftfreq(len(x), d=x[1]-x[0])

# Plot the frequency response

plt.figure(figsize=(10, 6))
plt.plot(freqs, np.abs(fft_y))

# Set title, axes and grid, limit
plt.title('Frequency Response of function')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Amplitude')
plt.grid(True)
plt.xlim(0, 200)

# Show the plot
plt.show()
