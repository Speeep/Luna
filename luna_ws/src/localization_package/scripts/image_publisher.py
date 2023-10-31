# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('image_publisher')
    image_publisher = rospy.Publisher('camera_image_topic', Image, queue_size=10)
    bridge = CvBridge()

    # Open the camera (e.g., using OpenCV)
    cap = cv2.VideoCapture(0)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Capture an image from the camera
        ret, frame = cap.read()

        if ret:
            # Convert the OpenCV image to a ROS Image message
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_publisher.publish(image_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
