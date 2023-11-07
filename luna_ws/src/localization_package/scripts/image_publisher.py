# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import cv2.aruco as aruco
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
import pickle

# Load the pickles
matrix_path = '/home/speeep/Development/Luna/luna_ws/src/localization_package/scripts/pkls/cameraMatrix.pkl'
dist_path = '/home/speeep/Development/Luna/luna_ws/src/localization_package/scripts/pkls/dist.pkl'

with open(matrix_path, 'rb') as file:
    camera_matrix = pickle.load(file)

with open(dist_path, 'rb') as file:
    dist = pickle.load(file)

# Define ArUco Characteristics
MARKER_SIZE = 19 # Centimeters TODO: Change if we make marker bigger

# Load the ArUco dictionary
dictionary = aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)


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

def main():
    rospy.init_node('image_publisher')
    image_publisher = rospy.Publisher('camera_image_topic', Image, queue_size=10)
    pose_publisher = rospy.Publisher('aruco_pose', Pose, queue_size=10)
    servo_error_publisher = rospy.Publisher('servo_error', Int32, queue_size=10)

    bridge = CvBridge()

    # Define Camera to Use
    cam = cv.VideoCapture(2)

    # Define green color
    GREEN = (0, 255, 0)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Capture an image from the camera
        ret, frame = cam.read()

        if ret:

            # BGR 2 Gray
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # Detect ArUco markers in the frame.
            marker_corners, ids, _ = aruco.detectMarkers(gray_frame, dictionary)

            height, width, channels = frame.shape

            # Draw detected markers on the frame.
            if ids is not None:

                # Calculate Rotation and Translation
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, MARKER_SIZE, camera_matrix, dist
                )

                # Total number of markers detected
                num_markers = range(0, ids.size)

                for id, corners, i in zip(ids, marker_corners, num_markers):

                    cv.polylines(
                        frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = tuple(corners[0].ravel())
                    top_left = tuple(corners[1].ravel())
                    bottom_right = tuple(corners[2].ravel())
                    bottom_left = tuple(corners[3].ravel())

                    # Calculating the distance
                    distance = np.sqrt(
                        tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    )

                    # Draw the pose of the marker
                    point = cv.drawFrameAxes(frame, camera_matrix, dist, rVec[i], tVec[i], 4, 4)

                    cv.putText(
                        img=frame,
                        text=f"id: {ids[0]} Dist: {round(distance, 2)}",
                        org=top_right,
                        fontFace=cv.FONT_HERSHEY_PLAIN,
                        fontScale=1.3,
                        color=GREEN,
                        thickness=2,
                        lineType=cv.LINE_AA,
                    )
                    cv.putText(
                        img=frame,
                        text=f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                        org=bottom_right,
                        fontFace=cv.FONT_HERSHEY_PLAIN,
                        fontScale=1.0,
                        color=GREEN,
                        thickness=2,
                        lineType=cv.LINE_AA,
                    )


                # centroid = calculate_centroid(corners)
                # servo_error_publisher.publish((width//2) - centroid[0])


            # else: 
            #     servo_error_publisher.publish(0)

            # Convert the OpenCV image to a ROS Image message
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_publisher.publish(image_message)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
