import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def depth_image_callback(msg):
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Apply colormap to visualize depth information
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Display the depth image with colormap
    cv2.imshow("Depth Image", depth_colormap)
    cv2.waitKey(1)

def main():
    rospy.init_node('depth_image_subscriber', anonymous=True)

    # Subscribe to the depth image topic
    rospy.Subscriber('/realsense/depth/image_raw', Image, depth_image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()