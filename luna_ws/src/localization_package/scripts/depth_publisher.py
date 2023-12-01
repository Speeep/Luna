import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
# import librealsense2 as lib

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

color_res = [640, 480]

depth_res = [640, 480]

def main():
    rospy.init_node('realsense_depth_publisher', anonymous=True)

    # Initialize RealSense pipeline
    pipeline = rs.pipeline()

    # Align
    align_to = rs.stream.color
    align = rs.align(align_to)

    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    #creates pointcloud reference
    pc = rs.pointcloud()
    colorizer = rs.colorizer()

    # Start streaming
    profile = pipeline.start(config)

    # Set up ROS publisher
    image_pub = rospy.Publisher('/realsense/depth/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    depth_sensor = profile.get_device().first_depth_sensor()


    if depth_sensor.supports(rs.option.depth_units):
        depth_sensor.set_option(rs.option.depth_units,0.0001)

        depth_sensor.set_option(rs.option.laser_power, 360)

    try:
        while not rospy.is_shutdown():
            # Wait for the next set of frames
            frames = pipeline.wait_for_frames()

            aligned_frames = align.process(frames)

            # Get depth frame
            depth_frame = aligned_frames.get_depth_frame()

            # Get color frame
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame:
                continue
            

            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)

            vtx = np.asanyarray(points.get_vertices())
            tex = np.asanyarray(points.get_texture_coordinates())
            
            # print(type(points), points)
            # print(type(vtx), vtx.shape, vtx)
            # print(type(tex), tex.shape, tex)

            # Convert depth frame to numpy array
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_cm = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            key = cv2.waitKey(1)
            
            if key == ord('d'):

                min_distance = 1e-6

                vtx = points.get_vertices()

                vertices = np.asanyarray(vtx).view(np.float32).reshape(-1,3) #Reshaping for xyz

                h,w,_ = color_image.shape

                rw_cords=[]

                counter = 0

                for i in range(points.size()):

                    rw_cords.append(vertices[i])

                # print("Number of pixels ignored:", counter)

                # pos = lib.pass_coordinate(depth_res[0]/2,depth_res[1]/2,depth_res[0])

                # print("SDK point:",

                #         "index =", pos,

                #         "x =", rw_cords[pos][0],

                #         "y =", rw_cords[pos][1],

                #         "z =", rw_cords[pos][2]

                #         ) 

            # Publish depth image to ROS
            depth_ros_msg = bridge.cv2_to_imgmsg(depth_cm, encoding="passthrough")
            depth_ros_msg.header.stamp = rospy.Time.now()
            image_pub.publish(depth_ros_msg)

    except rospy.ROSInterruptException:
        pass

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == '__main__':
    main()