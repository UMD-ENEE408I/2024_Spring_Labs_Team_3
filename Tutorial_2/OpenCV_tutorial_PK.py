# OpenCV tutorial
# Peter Krepkiy
# ENEE408I lab 2

import cv2

image = cv2.imread('Image.jpg')

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Display the original and grayscale images
cv2.imshow('Original Image', image)
cv2.imshow('Grayscale Image', gray_image)


# Problem (2)

edges = cv2.Canny(gray_image, 100, 200)

# Display the original and edge-detected images

cv2.imshow('Edge-detected Image', edges)


# Problem (3)


face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


# Perform face detection
faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

# Draw rectangles around the detected faces
for (x, y, w, h) in faces:
    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

# Display the image with detected faces
cv2.imshow('Detected Faces', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Homework: Explain how Harr cascades work to detect faces

# A haar cascade classifies features as either negative or positive. In this case it works as a binary classifier. The algorithm uses
# Edge, line and four-rectangle features. The sum of the classified pixels is calculated. The more features are applied to an image,
# the better accuracy to detect something in the image (i.e. a human face).