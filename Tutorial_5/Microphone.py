# Tutorial 5
# Peter Krepkiy
# S2024 Lab team 3
# Microphone and spectrogram 


import pyaudio
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Parameters for recording
duration = 5  # seconds
sample_rate = 44100  # Hz
chunk_size = 1024  # samples

# Initialize PyAudio
audio = pyaudio.PyAudio()

# Open stream
stream = audio.open(format=pyaudio.paFloat32,
                     channels=1,
                     rate=sample_rate,
                     input=True,
                     frames_per_buffer=chunk_size)

# Record audio
print("Recording...")
frames = []
for _ in range(0, int(sample_rate / chunk_size * duration)):
    data = stream.read(chunk_size)
    frames.append(np.frombuffer(data, dtype=np.float32))
print("Recording done")

# Close stream
stream.stop_stream()
stream.close()
audio.terminate()

# Combine recorded frames
audio_data = np.concatenate(frames)

# Calculate spectrogram
frequencies, times, spectrogram = signal.spectrogram(audio_data, sample_rate)

# Plot spectrogram
plt.pcolormesh(times, frequencies, 10 * np.log10(spectrogram))
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [s]')
plt.title('Spectrogram')
plt.colorbar(label='Intensity [dB]')
plt.show()
