import cv2 as cv
import numpy
import cv2.aruco as aruco
from cv2 import VideoCapture


cam = cv.VideoCapture(0)

GREEN = (0, 255, 0)

# Check if the camera is connected

if not cam.isOpened():
    print("Camera not found. Make sure your webcam is connected.")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cam.read()

    # Convert the frame to grayscale
    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    if not ret:
        print("Failed to grab frame.")
        break

    # Load the ArUco dictionary
    dictionary = aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

    # Detect ArUco markers in the frame.
    corners, ids, _ = aruco.detectMarkers(frame, dictionary)

    # Draw detected markers on the frame.
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids, GREEN)

    # display the image
    cv.imshow('frame.jpg', frame)

    # Check for the 'q' key to exit the loop and close the window.
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

    # cv.imshow('Image with ArUco Markers', frame)
    # ArUco marker detection parameters object
    # parameters = aruco.DetectorParameters_create()

    # __, frame = cam.read()
    # cv2.imshow('Image with ArUco Markers', frame)

    # Detect ArUco markers in the frame
    # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)

    # if ids is not None:
    #     # Draw rectangles around the detected markers with green borders
    #     frame_with_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids, borderColor=(0, 255, 0))

    #     # Display the frame with markers
    #     cv.imshow('Image with ArUco Markers', frame_with_markers)

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# Release the camera and close all OpenCV windows
print("no")
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

