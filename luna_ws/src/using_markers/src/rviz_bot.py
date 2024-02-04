#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
import tf.transformations

x = 0.0
y = 0.0
theta = 0.0

def filtered_pose_callback(msg):
    global x, y, theta
    # convert from cm to meters for x and y, theta is already in rad
    x = msg.data[0] / 100
    y = msg.data[1] / 100
    theta = msg.data[2]

def main():
    rospy.init_node("rviz_bot")
    marker_pub = rospy.Publisher("rviz_bot_marker", Marker, queue_size=1)
    rospy.Subscriber("jetson/filtered_pose", Float32MultiArray, filtered_pose_callback)
    rate = rospy.Rate(10)

    global x, y, theta

    # Set our initial shape type to be a cube
    shape = Marker.ARROW

    while not rospy.is_shutdown():
        marker = Marker()
        # Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "my_frame"
        marker.header.stamp = rospy.Time.now()

        # Set the namespace and id for this marker.  This serves to create a unique ID
        # Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "rviz_bot"
        marker.id = 0

        # Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape

        # Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = Marker.ADD

        # Set the pose of the robot using the filtered x, y, theta values
        marker.pose.position.x = x
        marker.pose.position.y = y
        yaw = theta

        print("x: " + str(x))
        print("y: " + str(y))
        print("yaw: " + str(yaw))

        # Convert Euler angles to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.pose.position.z = 0.05

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.3
        marker.scale.y = 0.15
        marker.scale.z = 0.05

        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()

        # Publish the marker
        while marker_pub.get_num_connections() < 1:
            if rospy.is_shutdown():
                return
            rospy.logwarn_once("Please create a subscriber to the marker")
            rospy.sleep(1)
        marker_pub.publish(marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass