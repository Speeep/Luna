import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def depth_image_callback(msg):
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Display the depth image
    cv2.imshow("Depth Image", depth_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('depth_image_subscriber', anonymous=True)

    # Subscribe to the depth image topic
    rospy.Subscriber('/realsense/depth/image_raw', Image, depth_image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()