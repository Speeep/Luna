import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from math import cos, sin

webcam_theta = 0

y_rot = np.array(
    [
        [0.5253, 0, 0.8509],
        [0, 1, 0],
        [-0.8509, 0, 0.5253]
    ]
)

z_rot = np.array(
    [
        [cos(webcam_theta), -sin(webcam_theta), 0],
        [sin(webcam_theta), cos(webcam_theta), 0],
        [0, 0, 1]
    ]
)

print()

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