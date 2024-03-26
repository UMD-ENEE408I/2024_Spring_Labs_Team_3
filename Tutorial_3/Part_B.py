# Peter Krepkiy
# ENEE408I lab 3, team 3
# Part B

# RMS, Cross-correlation, Sound Localization

import math
import scipy
import matplotlib.pyplot as plt
import numpy as np

from scipy.io import wavfile

plt.close('all')


# Problem 1

sample_rate_M1, M1_data = wavfile.read("M1.wav")

sample_rate_M2, M2_data = wavfile.read("M2.wav")

sample_rate_M3, M3_data = wavfile.read("M3.wav")

# Calculate the RMS value

print("RMS value of M1:",np.sqrt(np.mean(M1_data.astype(float)**2)))

print("RMS value of M2:",np.sqrt(np.mean(M2_data.astype(float)**2)))

print("RMS value of M3:",np.sqrt(np.mean(M3_data.astype(float)**2)))

# Problem 2

# Microphone M1 is closer to the sound source, since the RMS value of M1 is greater than
# the RMS value of M2.

# Problem 3

# CREDIT FOR CODE: GERSHOM

def cross_correlation(signal1, signal2):
    lag_array = np.zeros(signal2.shape[0]-1)
    signal2_array = np.concatenate((lag_array,signal2,lag_array)) 
    signal2_matrix = np.zeros((signal2.shape[0],2*signal2.shape[0]-1))

    for i in range(2*signal2.shape[0]-1):
        signal2_matrix[:, i]= signal2_array[i:i+m2_audio.shape[0]]

    signal1.reshape((1,signal1.shape[0]))
    correlation = np.matmul(signal1, signal2_matrix)
    return correlation 

m1_sampling_rate, m1_audio = wavfile.read("M1.wav")
m2_sampling_rate, m2_audio = wavfile.read("M2.wav")

correlation = cross_correlation(m1_audio, m2_audio)

plt.figure(figsize=(10, 4))
plt.title("Correlation")
plt.xlabel("Time (s)")
plt.ylabel("Correlation")
plt.plot(np.abs(correlation))
plt.grid()


sample_delay = np.argmax(np.abs(correlation))

time_delay  = (sample_delay-m1_audio.shape[0])  /m2_sampling_rate

print("Time delay: " + str(time_delay))

plt.show()





   