# Import necessary libraries
import rospy
import cv2

# def image_callback(image_msg):
#     bridge = CvBridge()
#     # Convert the ROS Image message to an OpenCV image
#     image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

#     # Display the image using OpenCV
#     cv2.imshow("Camera Image", image)
#     cv2.waitKey(1)

def main():
    rospy.init_node('robot')
    # rospy.Subscriber('camera_image_topic', Image, image_callback)
    print("I am a robot!")
    rospy.spin()

if __name__ == '__main__':
    print("I am a robot!")
    main()