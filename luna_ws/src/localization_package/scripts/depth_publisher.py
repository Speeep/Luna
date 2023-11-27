import rospy
import cv2
import numpy as np
import pyrealsense2 as rs

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('realsense_depth_publisher', anonymous=True)

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    # Set up ROS publisher
    image_pub = rospy.Publisher('/realsense/depth/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    try:
        while not rospy.is_shutdown():
            # Wait for the next set of frames
            frames = pipeline.wait_for_frames()

            # Get depth frame
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            # Convert depth frame to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())

            # Publish depth image to ROS
            depth_ros_msg = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
            depth_ros_msg.header.stamp = rospy.Time.now()
            image_pub.publish(depth_ros_msg)

    except rospy.ROSInterruptException:
        pass

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == '__main__':
    main()