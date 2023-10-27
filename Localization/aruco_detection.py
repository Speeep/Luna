import cv2 as cv
import numpy
import cv2.aruco as aruco
from cv2 import VideoCapture


# Define Camera to Use
cam = cv.VideoCapture(0)

# Define green color
GREEN = (0, 255, 0)

# Function to clean corners
def clean_corners(corners):
    clean_corners = []
    corners = (corners[0][0][0], corners[0][0][1], corners[0][0][2], corners[0][0][3])
    for corner in corners:
        corner = (corner[0], corner[1])
        clean_corners.append(corner)
    return clean_corners

# Check if the camera is connected
if not cam.isOpened():
    print("Camera not found. Make sure your webcam is connected.")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cam.read()

    if not ret:
        print("Failed to grab frame.")
        break

    # Load the ArUco dictionary
    dictionary = aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

    # Detect ArUco markers in the frame.
    corners, ids, _ = aruco.detectMarkers(frame, dictionary)

    # Draw detected markers on the frame.
    if ids is not None:
        corners = clean_corners(corners=corners)
        print(type(corners[0]))
        print(corners[0])
        cv.line(frame, corners[0], corners[1], GREEN, 2)
        cv.line(frame, corners[1], corners[2], GREEN, 2)
        cv.line(frame, corners[2], corners[3], GREEN, 2)
        cv.line(frame, corners[3], corners[0], GREEN, 2)


    # display the image
    cv.imshow('frame.jpg', frame)

    # Check for the 'q' key to exit the loop and close the window.
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cam.release()
cv.destroyAllWindows()

# corners: A list of NumPy arrays, where each NumPy array represents the corners of a detected marker.
# Each NumPy array has the shape (4, 2), where:
# The first dimension (4) represents the four corners of the marker.
# The second dimension (2) represents the (x, y) coordinates of each corner in the image.

# corners[0][0] - first corner and x axis ====== corners[0][1] - first corner and y axis
# corner indexes:
# 0 Top-Left Corner
# 1 Top-Right Corner
# 2 Bottom-Right Corner
# 3 Bottom-Left Corner


############################ Possible helpful functions ###########################

# cv2.waitKey(0)
# cv2.destroyAllWindows()

