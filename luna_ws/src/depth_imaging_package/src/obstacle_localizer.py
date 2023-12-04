import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from math import cos, sin, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

webcam_theta = 0.785398 # 45 Deg in Radians

transformation_matrix = np.array([
    [cos(webcam_theta - 1.5708), (sqrt(2)/2)*sin(webcam_theta - 1.5708), -(sqrt(2)/2)*sin(webcam_theta - 1.5708), -(8.76*sin(webcam_theta - 1.5708))],
    [sin(webcam_theta - 1.5708), -(sqrt(2)/2)*cos(webcam_theta - 1.5708), (sqrt(2)/2)*cos(webcam_theta - 1.5708), 8.76*cos(webcam_theta - 1.5708)],
    [0, -sqrt(2)/2, -sqrt(2)/2, -6.91],
    [0, 0, 0, 1]
])

print("Transformation Matrix:")
print(transformation_matrix)

def update_obstacle_pose(data):
    print(f'Obstacle Pose in Realsense Frame is: {data.data}')

def update_webcam_theta(data):
    webcam_theta = data.data
    print(f'Webcam theta: {webcam_theta}')

def listener():

    rospy.init_node('obstacle_localizer', anonymous=True)

    rospy.Subscriber('obstacle_pose_realsense_frame', Float32MultiArray, update_obstacle_pose)

    rospy.spin()

if __name__ == '__main__':
    listener()