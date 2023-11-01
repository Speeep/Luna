# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy
import cv2.aruco as aruco
from std_msgs.msg import Int32


# Function to clean corners
def clean_corners(corners):
    clean_corners = []
    corners = (corners[0][0][0], corners[0][0][1], corners[0][0][2], corners[0][0][3])
    for corner in corners:
        corner = (corner[0], corner[1])
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
    servo_error_publisher = rospy.Publisher('servo_error', Int32, queue_size=10)

    bridge = CvBridge()

    # Define Camera to Use
    cam = cv.VideoCapture(0)

    # Define green color
    GREEN = (0, 255, 0)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Capture an image from the camera
        ret, frame = cam.read()

        if ret:

            # Load the ArUco dictionary
            dictionary = aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

            # Detect ArUco markers in the frame.
            corners, ids, _ = aruco.detectMarkers(frame, dictionary)

            # Draw detected markers on the frame.
            if ids is not None:
                corners = clean_corners(corners=corners)
                centroid = calculate_centroid(corners)
                cv.line(frame, corners[0], corners[1], GREEN, 2)
                cv.line(frame, corners[1], corners[2], GREEN, 2)
                cv.line(frame, corners[2], corners[3], GREEN, 2)
                cv.line(frame, corners[3], corners[0], GREEN, 2)
                cv.circle(frame, (centroid[0], centroid[1]), 5, GREEN, -1)
                servo_error_publisher.publish(centroid[0])

            # Convert the OpenCV image to a ROS Image message
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_publisher.publish(image_message)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
