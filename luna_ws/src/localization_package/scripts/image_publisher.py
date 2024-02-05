# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from math import sin, cos
import cv2.aruco as aruco
from std_msgs.msg import Float32, Float32MultiArray
import pickle
from scipy.spatial.transform import Rotation as R

# Load the pickles
matrix_path = '/home/jetson/Development/Luna/luna_ws/src/localization_package/scripts/pkls027/cameraMatrix.pkl'
dist_path = '/home/jetson/Development/Luna/luna_ws/src/localization_package/scripts/pkls027/dist.pkl'

with open(matrix_path, 'rb') as file:
    camera_matrix = pickle.load(file)

with open(dist_path, 'rb') as file:
    dist = pickle.load(file)

# Define ArUco Characteristics
MARKER_SIZE = 88 # Centimeters TODO: Change if we make marker bigger

# Load the ArUco dictionary
dictionary = aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)

xws = []
yws = []

# Angle of the localizer turret, used in the transformation matrix to go from webcam to robot pose
localizer_angle = 0.0

# Function to clean corners
def clean_corners(corners):
    clean_corners = []
    corners = (corners[0][0][0], corners[0][0][1], corners[0][0][2], corners[0][0][3])
    for corner in corners:
        corner = [corner[0], corner[1]]
        clean_corners.append(corner)
    return clean_corners

# Function to calculate the centroid of 4 points
def calculate_centroid(points):
    if len(points) != 4:
        raise ValueError("Input list must contain exactly 4 (x, y) pairs.")
    
    # Calculate the sum of x and y coordinates
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    
    # Calculate the centroid coordinates
    centroid_x = int(sum_x / 4)
    centroid_y = int(sum_y / 4)
    
    return (centroid_x, centroid_y)

def update_localizer_angle_cb(localizer_angle_msg):
    global localizer_angle
    localizer_angle = localizer_angle_msg.data

def main():
    rospy.init_node('image_publisher')
    image_publisher = rospy.Publisher('camera_image_topic', Image, queue_size=10)
    servo_error_publisher = rospy.Publisher('/localizer/error', Float32, queue_size=10)
    aruco_data_publisher = rospy.Publisher('/jetson/localization_estimate', Float32MultiArray, queue_size=10)
    rospy.Subscriber('/jetson/localizer_angle', Float32, update_localizer_angle_cb)

    global localizer_angle

    bridge = CvBridge()

    # Define Camera to Use
    cam = cv.VideoCapture(0)

    # Define green color
    YELLOW = (0, 255, 255)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Capture an image from the camera
        ret, frame = cam.read()

        if ret:

            # BGR 2 Gray
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # Blur to remove noise
            gray_frame = cv.blur(gray_frame, (5,5))

            # Threshold image to get better ArUco detection
            threshold_value = 80
            thresh_ret, gray_frame = cv.threshold(gray_frame, threshold_value, 255, cv.THRESH_BINARY)

            frame = cv.cvtColor(gray_frame, cv.COLOR_GRAY2BGR)

            # cv.imshow('gray', gray_frame)
            # cv.waitKey(0)

            # Detect ArUco markers in the frame.
            marker_corners, ids, _ = aruco.detectMarkers(gray_frame, dictionary)
            height, width, channels = frame.shape

            # Draw detected markers on the frame.
            if ids is not None:

                # Find the index of the marker with ID 42
                index_42 = np.where(ids == 256)[0]

                if len(index_42) > 0:
                    i = index_42[0]  # Use the first occurrence of marker with ID 42

                    # Calculate Rotation and Translation for the selected marker
                    rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                        [marker_corners[i]], MARKER_SIZE, camera_matrix, dist
                    )

                    # Convert rotation vector to rotation matrix using Rodrigues' rotation formula
                    rotation_matrix, _ = cv.Rodrigues(rVec)

                    # Extract the yaw from the rotation matrix
                    theta = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))

                    # Extract x, y, and z from the translation vector
                    x, y, z = tVec[0][0][0], tVec[0][0][1], tVec[0][0][2]

                    distance = round(np.sqrt(x**2 + y**2 + z**2), 1)

                    xw = round((z * cos(theta) - x * sin(theta)), 1)
                    yw = round(abs((x * cos(theta) - z * sin(theta))), 1)

                    if len(xws) >= 100:
                        xws.pop(0)

                    if len(yws) >= 100:
                        yws.pop(0)

                    xws.append(xw)
                    yws.append(yw)

                    avg_xw = round(sum(xws) / len(xws), 1)
                    avg_yw = round(sum(yws) / len(yws), 1)

                    # Publish Theta to 'aruco_theta'
                    aruco_data_msg = Float32MultiArray()
                    aruco_data_msg.data = [avg_xw, avg_yw, theta]
                    aruco_data_publisher.publish(aruco_data_msg)

                    cv.polylines(
                        frame, [marker_corners[i].astype(np.int32)], True, YELLOW, 4, cv.LINE_AA
                    )

                    corners = marker_corners[i].reshape(4, 2)
                    corners = corners.astype(int)

                    # Draw the pose of the marker
                    point = cv.drawFrameAxes(frame, camera_matrix, dist, rVec[0], tVec[0], 4, 4)

                    # Define the starting and ending points of the line
                    start_point = (frame.shape[1] // 2, 0)
                    end_point = (frame.shape[1] // 2, frame.shape[0])
                    RED = (0, 0, 255)
                    cv.line(frame, start_point, end_point, RED, 1)

                    centroid = calculate_centroid(corners)
                    servo_error = int((width//2) - centroid[0])
                    servo_error_publisher.publish(servo_error)

            else: 
                servo_error_publisher.publish(0)

            # Convert the OpenCV image to a ROS Image message
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_publisher.publish(image_message)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
