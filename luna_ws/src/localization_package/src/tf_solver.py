import rospy
import tf2_ros
import tf.transformations, tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Float32


def update_localizer_angle_cb(localizer_angle_msg):
    global localizer_angle
    localizer_angle = localizer_angle_msg.data

if __name__ == '__main__':
    rospy.init_node('tf_solver')
    rospy.Subscriber('/jetson/localizer_angle', Float32, update_localizer_angle_cb)

    localizer_angle = 0.0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Define tf between (0, 0, 0) of World Frame and Aruco Marker
    world_2_aruco = TransformStamped()
    world_2_aruco.header.frame_id = "world"
    world_2_aruco.child_frame_id = "aruco"
    world_2_aruco.header.stamp = rospy.Time.now()
    world_2_aruco.transform.translation.x = 0.0
    world_2_aruco.transform.translation.y = 500.0
    world_2_aruco.transform.translation.z = 0.0
    world_2_aruco.transform.rotation.x = 0.0
    world_2_aruco.transform.rotation.y = 0.0
    world_2_aruco.transform.rotation.z = 0.0
    world_2_aruco.transform.rotation.w = 1.0

    # Define tf between webcam and robot
    webcam_2_robot = TransformStamped()
    webcam_2_robot.header.frame_id = "webcam"
    webcam_2_robot.child_frame_id = "robot"
    webcam_2_robot.header.stamp = rospy.Time.now()
    webcam_2_robot.transform.translation.x = -3.81
    webcam_2_robot.transform.translation.y = -12.192
    webcam_2_robot.transform.translation.z = 0.0
    webcam_2_robot.transform.rotation.x = 0.0
    webcam_2_robot.transform.rotation.y = 0.0
    webcam_2_robot.transform.rotation.z = 0.0
    webcam_2_robot.transform.rotation.w = 1.0

    # Define a basic point that we can apply all the tfs to
    robot_pose = PoseStamped()
    robot_pose.header = Header()
    robot_pose.header.stamp = rospy.Time.now()
    robot_pose.pose = Pose()
    robot_pose.pose.position = Point(0.0, 0.0, 0.0)
    robot_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    pose_pub = rospy.Publisher('/jetson/localizer_robot_pose', PoseStamped, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("aruco", "webcamTurned", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # Define tf between Aruco Marker and Webcam
        aruco_2_webcam_turned = trans.transform

        # Define tf between WebcamTurned and Webcam
        webcam_turned_2_webcam = TransformStamped()
        webcam_turned_2_webcam.header.frame_id = "webcamTurned"
        webcam_turned_2_webcam.child_frame_id = "webcam"
        webcam_turned_2_webcam.header.stamp = rospy.Time.now()
        webcam_turned_2_webcam.transform.translation.x = 0.0
        webcam_turned_2_webcam.transform.translation.y = 0.0
        webcam_turned_2_webcam.transform.translation.z = 0.0
        quat = tf.transformations.quaternion_from_euler(float(0.0),float(0.0),float(localizer_angle))
        webcam_turned_2_webcam.transform.rotation.x = quat[0]
        webcam_turned_2_webcam.transform.rotation.y = quat[1]
        webcam_turned_2_webcam.transform.rotation.z = quat[2]
        webcam_turned_2_webcam.transform.rotation.w = quat[3]

        # Sequentially apply the transforms to robot_pose in the following order

        # 1. world_2_aruco
        tf1 = tf2_geometry_msgs.do_transform_pose(robot_pose, world_2_aruco)

        # 2. aruco_2_webcam_turned
        tf2 = tf2_geometry_msgs.do_transform_pose(tf1, aruco_2_webcam_turned)

        # 3. webcam_turned_2_webcam
        tf3 = tf2_geometry_msgs.do_transform_pose(tf2, webcam_turned_2_webcam)

        # 4. webcam_2_robot
        robot_pose_final = tf2_geometry_msgs.do_transform_pose(tf3, webcam_2_robot)

        # Publish the final Robot Pose
        pose_pub.publish(robot_pose_final)

        rate.sleep()