# Numpy tutorial
# Peter Krepkiy
# ENEE408I lab 2

# Problem (1) Why is numpy faster than using for loops in Python for operations such as matrix multiplications?

# The array operations are done matrix-wise, not element wise, so it's faster to operate on the entire array as one object
# rather than each individual index in the array.


# Problem (2) What is the data type used in numpy which makes such operations feasible? Also name a few
# differences between this data type and its similar counterpart in Python

# Numpy uses an object called a homogeneous multidimensional array. a similar python counterpart is a python list, but the python
# list is less efficient. The Numpy array is homogeneous and is stored as a contiguous block in memory, whereas python lists are dynamically
# allocated, which makes them slower.

# Problem (3)

import numpy as np 

array = np.array([1,2,3,4])

print(array)

# Problem (4)

ones_array = np.ones((3, 4))

zeros_array = np.zeros((4, 3))

# Problem 5

# Create a 2x3 matrix
A_matrix = np.array([[1, 2, 3],[4, 5, 6]])


# Create a 3x4 matrix
B_matrix = np.array([[7, 8, 9, 10],[11, 12, 13, 14],[15, 16, 17, 18]])

# Matrix multiplication
result = np.dot(A_matrix, B_matrix)

print(result)

# Problem 6

problem6_matrix = np.array([[3, 1],[1, 2]])

eigenvalues, eigenvectors = np.linalg.eig(problem6_matrix)

print(eigenvalues)



