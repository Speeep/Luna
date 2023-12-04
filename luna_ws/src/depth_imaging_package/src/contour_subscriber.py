import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json

# # Example usage
# u_pixel = 100  # Replace with your pixel coordinates
# v_pixel = 150  # Replace with your pixel coordinates
# depth_value = 1.5  # Replace with your depth value at (u, v)

# baseline = float(calibration_data["baseline"])
# intrinsic_left = [float(calibration_data[f"intrinsic_left.{axis}.{component}"]) for axis in 'xyz' for component in 'xyz']
# rectified = [float(calibration_data[f"rectified.0.{param}"]) for param in ['fx', 'fy', 'ppx', 'ppy']]

# x, y, z = pixel_to_cartesian(u_pixel, v_pixel, depth_value, baseline, intrinsic_left, rectified)

# print(f"Cartesian Coordinates: x={x}, y={y}, z={z}")

def contour_image_callback(msg):
    bridge = CvBridge()
    contour_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Display the depth image with colormap
    cv2.imshow("Contour Image", contour_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('contour_image_subscriber', anonymous=True)

    # Subscribe to the depth image topic
    rospy.Subscriber('/realsense/depth/contour_image', Image, contour_image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()