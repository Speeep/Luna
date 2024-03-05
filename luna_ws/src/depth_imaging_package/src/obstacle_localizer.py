import rospy
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
from math import cos, sin, sqrt, pi

WEBCAM_HEIGHT = 1.17 # 117 cm

# Define global variables
WEBCAM_HEIGHT = 1.17 # 98 cm
aruco_theta = 0
webcam_theta = 0
x = 0
y = 0
obstacle_location_realsense = np.array([[0], [0], [0], [1]])

# Define transformation matrix
realsense_2_webcam_tf = np.array([
    [cos(webcam_theta - 1.5708), (sqrt(2)/2)*sin(webcam_theta - 1.5708), -(sqrt(2)/2)*sin(webcam_theta - 1.5708), -(0.0876*sin(webcam_theta - 1.5708))],
    [sin(webcam_theta - 1.5708), -(sqrt(2)/2)*cos(webcam_theta - 1.5708), (sqrt(2)/2)*cos(webcam_theta - 1.5708), 0.0876*cos(webcam_theta - 1.5708)],
    [0, -sqrt(2)/2, -sqrt(2)/2, -0.0691],
    [0, 0, 0, 1]
])

# Define callback functions
def update_obstacle_pose(data):
    global obstacle_location_realsense
    obstacle_location_realsense[0][0] = data.data[0]
    obstacle_location_realsense[1][0] = data.data[1]
    obstacle_location_realsense[2][0] = data.data[2]
    rad_m = data.data[3]
    owf = np.matmul(realsense_2_webcam_tf, obstacle_location_realsense)
    obstacle_location_webcam = np.array([owf[0][0], owf[1][0], owf[2][0], 1])

    # Solve for the theta value to rotate about z axis for world to webcam tf
    if aruco_theta > 0:
        webcam_2_world_theta = pi + aruco_theta
    elif aruco_theta < 0:
        webcam_2_world_theta = pi - aruco_theta
    else:
        webcam_2_world_theta = pi

    webcam_2_world_tf = webcam_2_world(webcam_2_world_theta, x, y, WEBCAM_HEIGHT)

    obstacle_location_world = np.dot(webcam_2_world_tf, obstacle_location_webcam)
    obstacle_location_world = np.round(obstacle_location_world, 3)

    obstacle = Float32MultiArray()
    obstacle.data = [obstacle_location_world[0], obstacle_location_world[1], rad_m]

    obstacle_pub.publish(obstacle)

def update_webcam_theta(data):
    global webcam_theta
    webcam_theta = data.data

def update_aruco_data(data):
    global x, y, aruco_theta
    x = data.data[0] * 0.01 # Convert cm to m
    y = data.data[1] * 0.01 # Convert cm to m
    aruco_theta = data.data[2]

# Define helper functions
def world_2_webcam(webcam_2_world_theta, x, y, webcam_height):
    translate = np.array([
        [1, 0, 0, -x],
        [0, 1, 0, -y],
        [0, 0, 1, -webcam_height],
        [0, 0, 0, 1]
    ])
    rotate = np.array([
        [cos(webcam_2_world_theta), -sin(webcam_2_world_theta), 0, 0],
        [sin(webcam_2_world_theta), cos(webcam_2_world_theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return rotate @ translate

def webcam_2_world(webcam_2_world_theta, x, y, WEBCAM_HEIGHT):
    return np.linalg.inv(world_2_webcam(webcam_2_world_theta, x, y, WEBCAM_HEIGHT))

# Initialize ROS node
rospy.init_node('obstacle_localizer', anonymous=True)

# Initialize publishers and subscribers
obstacle_pub = rospy.Publisher('jetson/localized_obstacle', Float32MultiArray, queue_size=10)
rospy.Subscriber('/realsense/depth/obstacle', Float32MultiArray, update_obstacle_pose)
rospy.Subscriber('/jetson/localizer_angle', Float32, update_webcam_theta)
rospy.Subscriber('/jetson/aruco_data', Float32MultiArray, update_aruco_data)

# Define timer callback function
def timer_callback(event):
    pass  # Do nothing here as publishing is handled in the update_obstacle_pose callback

# Create a timer with a callback function that triggers publishing
timer = rospy.Timer(rospy.Duration(0.1), timer_callback)

# Spin ROS node
rospy.spin()