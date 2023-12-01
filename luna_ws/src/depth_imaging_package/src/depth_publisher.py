import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Set up a queue for temporal smoothing
depth_buffer = deque(maxlen=5)

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

            # Apply temporal smoothing
            depth_buffer.append(depth_data)
            smoothed_depth_data = np.mean(depth_buffer, axis=0)

            # Publish smoothed depth image to ROS
            depth_ros_msg = bridge.cv2_to_imgmsg(smoothed_depth_data, encoding="passthrough")
            depth_ros_msg.header.stamp = rospy.Time.now()
            image_pub.publish(depth_ros_msg)

    except rospy.ROSInterruptException:
        pass

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == '__main__':
    main()