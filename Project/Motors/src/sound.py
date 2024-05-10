

import socket
import time



# SOUND STUFF

import pyaudio
import numpy as np
import wave
from scipy import signal
from scipy.fft import fft, rfft
from scipy.fft import fftfreq, rfftfreq
import matplotlib.pyplot as plt
from scipy.io import wavfile as wav
from scipy.signal import butter, filtfilt, lfilter

def butter_highpass(cutoff_freq, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def apply_highpass_filter(data, cutoff_freq, fs, order=5):
    b, a = butter_highpass(cutoff_freq, fs, order=order)
    filtered_data = lfilter(b, a, data)
    return filtered_data


# Parameters for audio recording
FORMAT = pyaudio.paInt16  # Sample format
CHANNELS = 1              # Number of audio channels (1 for mono)
RATE = 44100              # Sample rate (samples per second)
CHUNK = 2048              # Number of frames per buffer (increased chunk size)

# Initialize PyAudio
audio = pyaudio.PyAudio()

# Find USB microphone indices
def find_usb_microphone():
    idx = np.array([])
   
    for i in range(audio.get_device_count()):
        device_info = audio.get_device_info_by_index(i)
        if 'USB' in device_info['name']:
            idx = np.append(idx, i)
           
    return idx.astype(int)

usb_mic_indices = find_usb_microphone()

# Open audio streams with USB microphones
streamL = audio.open(format=FORMAT,
                     channels=CHANNELS,
                     rate=RATE,
                     input=True,
                     input_device_index=usb_mic_indices[0],
                     frames_per_buffer=CHUNK)


print("Recording...")

# List to store audio frames
framesL = []

# Record audio for a certain duration (e.g., 5 seconds)
RECORD_SECONDS = 5

for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data1 = streamL.read(CHUNK)
    framesL.append(np.frombuffer(data1, dtype=np.float32))
   

print("Finished recording.")

# Stop and close the audio streams
streamL.stop_stream()
streamL.close()

audio.terminate()

audio_data = np.concatenate(framesL)

# Save the recorded audio to a WAV file
output_file1 = "output1.wav"
with wave.open(output_file1, 'wb') as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(audio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(framesL))
print(f"Audio saved to {output_file1}")


rate, data = wav.read('output1.wav')
# Time = np.linspace(0, len(data) / rate, num=len(data))
# plt.plot(Time, data)
# plt.show()

filtered = apply_highpass_filter(data, 8500, 44100)
# yf = fft(data)
# N = len(data)
# normalize = N/2
# plt.plot(rfftfreq(N, d=1/RATE), 2*np.abs(rfft(data))/N)
# plt.title('Original Spectrum')
# plt.xlabel('Frequency[Hz]')
# plt.ylabel('Amplitude')
# plt.show()

# yf = fft(filtered)
N = len(filtered)
# normalize = N/2
# plt.plot(rfftfreq(N, d=1/RATE), 2*np.abs(rfft(filtered))/N)
# plt.title('Filtered Spectrum')
# plt.xlabel('Frequency[Hz]')
# plt.ylabel('Amplitude')
# plt.show()

Amplitude = max(np.abs(rfft(filtered))/(N/2))




# WIFI STUFF


host = '192.168.4.1'  # IP address of the Heltec v2 access point
port = 80  # Port number


# Connect to the server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))
print("Connected to Heltec v2 access point")

#for m in range(50):
        # Send a message to the server

client_socket.sendall(Amplitude.encode())
print("Sent message:", Amplitude)

        # **No need to wait for a response here**

time.sleep(3.5)  # Wait for 1 second before sending the next message



client_socket.close()