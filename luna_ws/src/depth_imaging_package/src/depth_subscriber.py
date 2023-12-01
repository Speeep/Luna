import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json

# File Path to RealSense Calibration Data
file_path = '/home/speeep/Development/Luna/luna_ws/src/depth_imaging_package/src/d455_calib_data'

with open(file_path, 'r') as file:
    calibration_data = json.load(file)

def pixel_to_cartesian(u, v, depth, baseline, intrinsic, rectified):
    # Intrinsic parameters
    fx = intrinsic[0]
    fy = intrinsic[1]
    cx = intrinsic[2]
    cy = intrinsic[3]

    # Rectified parameters
    ppx = rectified[0]
    ppy = rectified[1]

    # Convert pixel coordinates to normalized image coordinates
    x_normalized = (u - ppx) / fx
    y_normalized = (v - ppy) / fy

    # Convert normalized image coordinates to rectified camera coordinates
    x_rectified = depth * x_normalized
    y_rectified = depth * y_normalized

    # Apply baseline to get disparity
    disparity = baseline / x_rectified

    # Convert rectified camera coordinates to Cartesian coordinates
    x_cartesian = disparity
    y_cartesian = disparity * y_normalized
    z_cartesian = disparity * depth

    return x_cartesian, y_cartesian, z_cartesian

# # Example usage
# u_pixel = 100  # Replace with your pixel coordinates
# v_pixel = 150  # Replace with your pixel coordinates
# depth_value = 1.5  # Replace with your depth value at (u, v)

# baseline = float(calibration_data["baseline"])
# intrinsic_left = [float(calibration_data[f"intrinsic_left.{axis}.{component}"]) for axis in 'xyz' for component in 'xyz']
# rectified = [float(calibration_data[f"rectified.0.{param}"]) for param in ['fx', 'fy', 'ppx', 'ppy']]

# x, y, z = pixel_to_cartesian(u_pixel, v_pixel, depth_value, baseline, intrinsic_left, rectified)

# print(f"Cartesian Coordinates: x={x}, y={y}, z={z}")

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
    rospy.Subscriber('/realsense/depth/image_aligned', Image, depth_image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()