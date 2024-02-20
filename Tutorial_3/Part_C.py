# Peter Krepkiy
# ENEE408I lab 3, team 3
# Part C

# Frequency spectrum

import math
import scipy
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy.signal import butter, lfilter

plt.close('all')


from scipy.io import wavfile

sample_rate, data = wavfile.read("Cafe_with_noise.wav")

duration = len(data) / sample_rate
time = np.linspace(0., duration, len(data))

# Plot the audio waveform
plt.figure(figsize=(12, 4))
plt.plot(time, data, color='black')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('Audio waveform')
plt.grid()
plt.show()

#Plot the frequency domain
fft_spectrum = np.fft.rfft(data)
freq = np.fft.rfftfreq(data.size, d=1./sample_rate)
fft_spectrum_abs = np.abs(fft_spectrum)


plt.plot(freq, fft_spectrum_abs)
plt.xlabel("frequency, Hz")
plt.ylabel("Amplitude, units")
plt.show()

def butter_lowpass(cutoff_freq, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def apply_lowpass_filter(data, cutoff_freq, fs, order=5):
    b, a = butter_lowpass(cutoff_freq, fs, order=order)
    filtered_data = lfilter(b, a, data)
    return filtered_data


newdata = apply_lowpass_filter(data, 15000, 48000)

wavfile.write("quietCafe.wav", sample_rate, newdata)
