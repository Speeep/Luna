import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Set up a queue for temporal smoothing
depth_buffer = deque(maxlen=5)

min_depth_difference = 20  # Minimum depth difference to consider as an object
canny_threshold1 = 30
canny_threshold2 = 100

def main():
    rospy.init_node('realsense_depth_publisher', anonymous=True)

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    # Start streaming
    pipeline.start(config)

    # Set up align object
    align = rs.align(rs.stream.color)

    # Set up ROS publisher
    image_pub = rospy.Publisher('/realsense/depth/image_aligned', Image, queue_size=10)
    contour_pub = rospy.Publisher('/realsense/depth/contour_image', Image, queue_size=10)
    bridge = CvBridge()

    try:
        while not rospy.is_shutdown():
            # Wait for the next set of frames
            frames = pipeline.wait_for_frames()

            # Align the depth frame to the color frame
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert depth frame to depth image
            depth_data = np.asanyarray(depth_frame.get_data())

            # Apply spatial smoothing (Gaussian blur)
            smoothed_depth_data = cv2.GaussianBlur(depth_data, (3, 3), 0)

            # Convert to 8-bit unsigned integer
            smoothed_depth_data = (smoothed_depth_data / np.max(smoothed_depth_data) * 255).astype(np.uint8)

            # Compute the gradient using the Canny edge detector
            edges = cv2.Canny(smoothed_depth_data, canny_threshold1, canny_threshold2)

            # Find contours in the edges
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

             # Convert color_frame to NumPy array
            color_frame_np = np.asanyarray(color_frame.get_data())

            # Filter contours based on depth difference
            for contour in contours:
                # Calculate the average depth within the contour
                mask = np.zeros_like(smoothed_depth_data, dtype=np.uint8)
                cv2.drawContours(mask, [contour], 0, 255, thickness=cv2.FILLED)
                avg_depth = np.mean(smoothed_depth_data[mask > 0])

                # Check if the depth difference is significant
                if np.abs(avg_depth - smoothed_depth_data[contour[0][0][1], contour[0][0][0]]) > min_depth_difference:
                    # Find the convex hull of the contour
                    convex_hull = cv2.convexHull(contour)

                    # Draw the convex hull on the color frame
                    cv2.drawContours(color_frame_np, [convex_hull], 0, (255, 0, 0), 2)

            # Apply temporal smoothing
            depth_buffer.append(depth_data)
            smoothed_depth_data = np.mean(depth_buffer, axis=0)

            # Publish smoothed depth image to ROS
            depth_ros_msg = bridge.cv2_to_imgmsg(smoothed_depth_data, encoding="passthrough")
            depth_ros_msg.header.stamp = rospy.Time.now()
            image_pub.publish(depth_ros_msg)

            # Publish contour BGR image to ROS
            contour_ros_msg = bridge.cv2_to_imgmsg(color_frame_np, encoding="bgr8")
            contour_ros_msg.header.stamp = rospy.Time.now()
            contour_pub.publish(contour_ros_msg)

    except rospy.ROSInterruptException:
        pass

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == '__main__':
    main()