import activate as activate
import cv2 as cv
import numpy
import cv2.aruco as aruco
from cv2 import VideoCapture
# from numpy.distutils.system_info import conda
from pip._internal.utils import virtualenv
# pip install --upgrade opencv-contrib-python
# pip install --upgrade opencv-python
# install coverage.py
# pip install opencv-contrib-python==4.6.0.66

# Create a VideoCapture object for your webcam (0 usually represents the default camera)

# cam = cv.VideoCapture(0)                                   #taken out for testing

# Check if the camera is connected

                                                            # taken out for testing
# if not cam.isOpened():
#     print("Camera not found. Make sure your webcam is connected.")
#     exit()                                                # til here

while True:
    # Read a frame from the camera
    # ret, frame = cam.read()                                   #taken out for testing

    # if not ret:
    #     print("Failed to grab frame.")
    #     break                                                    #til here

    # write the image
    # cv.imwrite('Current_image.jpg', frame)                       #taken out for testing

    frame = cv.imread('Current_image.jpg')

    # Convert the frame to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Load the ArUco dictionary
    # aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    dictionary = aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    refinedParameters = aruco.RefineParameters()
    detector = aruco.ArucoDetector(dictionary, parameters, refinedParameters)
    corners, ids, rejects = detector.detectMarkers(frame)

    # corners, ids, _ = aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)

    # corners, ids, rejectedCandidates = detector.detectMarkers(frame)
    #
    # cv.imshow('Image with ArUco Markers', frame)
    # ArUco marker detection parameters object
    # parameters = aruco.DetectorParameters_create()

    # __, frame = cam.read()
    # cv2.imshow('Image with ArUco Markers', frame)

    # Detect ArUco markers in the frame
    # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_frame, dictionary, parameters=parameters)

    if ids is not None:
        # Draw rectangles around the detected markers with green borders
        frame_with_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids, borderColor=(0, 255, 0))

        # Display the frame with markers
        cv.imshow('Image with ArUco Markers', frame_with_markers)

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

# aruco.estimatePoseSingleMarkers(): This function is used to estimate the pose (position and orientation) of
# individual ArUco markers in the image. It takes the detected marker corners and camera parameters as input and
# returns the rotation and translation vectors.

# aruco.drawAxis(): After estimating the pose of ArUco markers, you can use this function to draw 3D axes on the
# markers to represent their orientation in space.

# use charuco markers?

# aruco.drawDetectedCornersCharuco(): This function is specifically used for Charuco markers, which are a combination
# of chessboard and ArUco markers. It draws the detected corners of Charuco markers on an image.

# aruco.estimatePoseCharucoBoard(): This function estimates the pose of a Charuco board in the image. It takes the
# detected Charuco corners, board configuration, camera parameters, and distortion coefficients as input and returns
# the rotation and translation vectors.

# aruco.drawAxisCharuco(): Similar to drawAxis(), this function draws 3D axes on a Charuco board to represent its
# orientation in space.
