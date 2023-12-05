import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from math import cos, sin, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

webcam_theta = 0

realsense_2_webcam_tf = np.array([
    [cos(webcam_theta - 1.5708), (sqrt(2)/2)*sin(webcam_theta - 1.5708), -(sqrt(2)/2)*sin(webcam_theta - 1.5708), -(0.0876*sin(webcam_theta - 1.5708))],
    [sin(webcam_theta - 1.5708), -(sqrt(2)/2)*cos(webcam_theta - 1.5708), (sqrt(2)/2)*cos(webcam_theta - 1.5708), 0.0876*cos(webcam_theta - 1.5708)],
    [0, -sqrt(2)/2, -sqrt(2)/2, -0.0691],
    [0, 0, 0, 1]
])

obstacle_location_realsense = [
    [0],
    [0],
    [0],
    [1]
]

def update_obstacle_pose(data):
    obstacle_location_realsense[0][0] = data.data[0]
    obstacle_location_realsense[1][0] = data.data[1]
    obstacle_location_realsense[2][0] = data.data[2]
    obstacle_location_webcam = np.matmul(realsense_2_webcam_tf, obstacle_location_realsense)
    obstacle_location_webcam = np.round(obstacle_location_webcam, 3)
    print("Obstacle Location in Webcam Frame:")
    print(obstacle_location_webcam[0:3])

def update_webcam_theta(data):
    webcam_theta = data.data
    print(f'Webcam theta: {webcam_theta}')

def listener():

    rospy.init_node('obstacle_localizer', anonymous=True)

    rospy.Subscriber('/realsense/depth/obstacle', Float32MultiArray, update_obstacle_pose)

    rospy.spin()

if __name__ == '__main__':
    listener()