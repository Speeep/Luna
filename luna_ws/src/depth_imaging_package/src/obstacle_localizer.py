import rospy
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
from math import cos, sin, sqrt, pi

# Define global variables

REALSENSE_OFFSET_X = 0.125
REALSENSE_OFFSET_Y = 0.215
REALSENSE_OFFSET_Z = 1.08

x = 0
y = 0
theta = 0
obstacle_location_realsense = [0, 0, 0, 0]


# Define callback functions
def update_obstacle_pose(data):
    global obstacle_location_realsense, REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y, REALSENSE_OFFSET_Z
    obstacle_location_realsense = data.data

    point_realsense = np.array([obstacle_location_realsense[2], -obstacle_location_realsense[0], -obstacle_location_realsense[1], 1])

    realsense_to_robot_tf = np.array([
        [0.707, 0,  -0.707, REALSENSE_OFFSET_X],
        [0,     1,  0,      REALSENSE_OFFSET_Y],
        [0.707, 0,  0.707,  REALSENSE_OFFSET_Z],
        [0,     0,  0,      1]
    ])

    point_robot = np.dot(realsense_to_robot_tf, point_realsense)

    print("Obstace Location Robot: " + str(point_robot))

    # obstacle = Float32MultiArray()
    # obstacle.data = [obstacle_location_world[0], obstacle_location_world[1], rad_m]

    # obstacle_pub.publish(obstacle)

# Initialize ROS node
rospy.init_node('obstacle_localizer', anonymous=True)

# Initialize publishers and subscribers
obstacle_pub = rospy.Publisher('jetson/localized_obstacle', Float32MultiArray, queue_size=10)
rospy.Subscriber('/realsense/depth/obstacle', Float32MultiArray, update_obstacle_pose)

# Define timer callback function
def timer_callback(event):
    pass  # Do nothing here as publishing is handled in the update_obstacle_pose callback

# Create a timer with a callback function that triggers publishing
timer = rospy.Timer(rospy.Duration(0.1), timer_callback)

# Spin ROS node
rospy.spin()