import rospy
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
from math import cos, sin, sqrt, pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

WEBCAM_HEIGHT = 0.98 # 98 cm

aruco_theta = 0
webcam_theta = 0
x = 0
y = 0

realsense_2_webcam_tf = np.array([
    [cos(webcam_theta - 1.5708), (sqrt(2)/2)*sin(webcam_theta - 1.5708), -(sqrt(2)/2)*sin(webcam_theta - 1.5708), -(0.0876*sin(webcam_theta - 1.5708))],
    [sin(webcam_theta - 1.5708), -(sqrt(2)/2)*cos(webcam_theta - 1.5708), (sqrt(2)/2)*cos(webcam_theta - 1.5708), 0.0876*cos(webcam_theta - 1.5708)],
    [0, -sqrt(2)/2, -sqrt(2)/2, -0.0691],
    [0, 0, 0, 1]
])

def webcam_2_world(webcam_2_world_theta, x, y, webcam_height): 
    return np.array([
        [cos(webcam_2_world_theta), -sin(webcam_2_world_theta), 0, x],
        [sin(webcam_2_world_theta), cos(webcam_2_world_theta), 0, y],
        [0, 0, 1, webcam_height],
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
    if aruco_theta > 0:
        webcam_2_world_theta = pi - aruco_theta
    elif aruco_theta < 0:
        webcam_2_world_theta = pi + aruco_theta
    else:
        webcam_2_world_theta = pi
    print(aruco_theta)
    print(webcam_2_world_theta)
    print(webcam_2_world_theta * 180 / pi)
    webcam_2_world_tf = webcam_2_world(webcam_2_world_theta, x, y, WEBCAM_HEIGHT)
    print(webcam_2_world_tf)
    obstacle_location_world = np.matmul(webcam_2_world_tf, obstacle_location_webcam)
    obstacle_location_world = np.round(obstacle_location_world, 3)

    print("Obstacle Location in World Frame:")
    print(obstacle_location_world[0:3])

def update_webcam_theta(data):
    webcam_theta = data.data
    print(f'Webcam theta: {webcam_theta}')

def update_aruco_data(data):
    global x, y, aruco_theta
    x = data.data[0] * 0.001 # Convert cm to m
    y = data.data[1] * 0.001 # Convert cm to m
    aruco_theta = data.data[2]
    print(f'ARUCO DATA X: {x}, Y: {y}, Theta: {aruco_theta}')

def obstacle_localizer():

    rospy.init_node('obstacle_localizer', anonymous=True)

    rospy.Subscriber('/realsense/depth/obstacle', Float32MultiArray, update_obstacle_pose)
    rospy.Subscriber('aruco_data', Float32MultiArray, update_aruco_data)

    rospy.spin()

if __name__ == '__main__':
    obstacle_localizer()