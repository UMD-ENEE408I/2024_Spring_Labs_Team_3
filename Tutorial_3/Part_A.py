# Peter Krepkiy
# ENEE408I lab 3, team 3
# Part A


import scipy.io
import matplotlib.pyplot as plt
import numpy as np

from scipy.io import wavfile

plt.close('all')


# Problem 1, 2

# Read the .wav file
sample_rate, audio_data = wavfile.read("human_voice.wav")

print("The sample rate is",sample_rate,"Hz")


duration = len(audio_data) / sample_rate

# print(len(audio_data))

# Problem 3

# Create a time array
time = np.arange(0, duration, 1/sample_rate)

# Plot the original signal
plt.figure(figsize=(10, 4))
plt.plot(time, audio_data, color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('human_voice.wav')
plt.grid(True)


# Downsample the audio file (Problem 4)


new_sample_rate = 8000

# Integer downsampling factor
downsampling_factor = sample_rate // new_sample_rate

# Downsample the audio data
downsampled_audio = audio_data[::downsampling_factor]

# Save the downsampled audio to a new .wav file

wavfile.write('downsampled_audio.wav', new_sample_rate, downsampled_audio)

# Problem 5

print(len(downsampled_audio)," samples were obtained after downsampling.")

# Problem 6

# Plot the original signal
plt.figure(figsize=(10, 4))
plt.plot(time, audio_data, color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('downsampled_audio.wav')
plt.grid(True)

# Question 7

# The samples are further apart from each other in the downsampled audio file. There are also less overall number of samples,
# by a factor of 6 (48000/ 8000 Hz)
# This is consistent with the process of 'decimation' of a signal

plt.show()
