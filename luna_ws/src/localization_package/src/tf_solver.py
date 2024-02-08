import rospy
import tf2_ros
import tf.transformations, tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Float32
import numpy


def update_localizer_angle_cb(localizer_angle_msg):
    global localizer_angle
    localizer_angle = localizer_angle_msg.data

def multiply_transforms(trans1, trans2):
    # Convert TransformStamped messages to matrices
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(trans1)
    mat1 = numpy.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(trans2)
    mat2 = numpy.dot(trans2_mat, rot2_mat)

    mat3 = numpy.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    quat3 = tf.transformations.quaternion_from_matrix(mat3)

    # Convert the resulting matrix back to a TransformStamped message
    result_trans = TransformStamped()
    result_trans.header.frame_id = trans1.header.frame_id
    result_trans.child_frame_id = trans2.child_frame_id
    result_trans.transform.translation.x = trans3.translation.x
    result_trans.transform.translation.y = trans3.translation.y
    result_trans.transform.translation.z = trans3.translation.z
    result_trans.transform.rotation.x = quat3[0]
    result_trans.transform.rotation.y = quat3[1]
    result_trans.transform.rotation.z = quat3[2]
    result_trans.transform.rotation.w = quat3[3]

    return result_trans

if __name__ == '__main__':
    rospy.init_node('tf_solver')
    rospy.Subscriber('/jetson/localizer_angle', Float32, update_localizer_angle_cb)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    localizer_angle = 0.0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Define tf between (0, 0, 0) of World Frame and Aruco Marker
    world_2_aruco = TransformStamped()
    world_2_aruco.header.frame_id = "world"
    world_2_aruco.child_frame_id = "aruco"
    world_2_aruco.header.stamp = rospy.Time.now()
    world_2_aruco.transform.translation.x = 0.0
    world_2_aruco.transform.translation.y = 5.0
    world_2_aruco.transform.translation.z = 0.0
    world_2_aruco.transform.rotation.x = 0.0
    world_2_aruco.transform.rotation.y = 0.0
    world_2_aruco.transform.rotation.z = 0.0
    world_2_aruco.transform.rotation.w = 1.0

    # Define tf between webcam and robot
    webcam_2_robot = TransformStamped()
    webcam_2_robot.header.frame_id = "webcam"
    webcam_2_robot.child_frame_id = "robot_unfused"
    webcam_2_robot.header.stamp = rospy.Time.now()
    webcam_2_robot.transform.translation.x = -0.0381
    webcam_2_robot.transform.translation.y = -0.12192
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
    aruco_pose_pub = rospy.Publisher('/rviz/aruco_marker_pose', PoseStamped, queue_size=10)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            # Define tf between Aruco Marker and Webcam
            aruco_2_webcam_turned = tfBuffer.lookup_transform("aruco", "webcamTurned", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

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
        aruco_marker_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, world_2_aruco)

        aruco_pose_pub.publish(aruco_marker_pose)

        # 2. aruco_2_webcam_turned
        webcam_turned_pose = tf2_geometry_msgs.do_transform_pose(aruco_marker_pose, aruco_2_webcam_turned)

        # 3. webcam_turned_2_webcam
        webcam_pose = tf2_geometry_msgs.do_transform_pose(webcam_turned_pose, webcam_turned_2_webcam)

        # 4. webcam_2_robot
        robot_pose_final = tf2_geometry_msgs.do_transform_pose(webcam_pose, webcam_2_robot)

        world_2_webcam_turned = multiply_transforms(world_2_aruco, aruco_2_webcam_turned)

        print("tf solver world_2_webcam_turned x: " + str(world_2_webcam_turned.transform.translation.x))
        print("tf solver world_2_webcam_turned y: " + str(world_2_webcam_turned.transform.translation.y))

        # Publish the final Robot Pose
        pose_pub.publish(robot_pose_final)

        broadcaster.sendTransform(world_2_aruco)
        broadcaster.sendTransform(aruco_2_webcam_turned)
        broadcaster.sendTransform(webcam_turned_2_webcam)
        broadcaster.sendTransform(webcam_2_robot)
        broadcaster.sendTransform(world_2_webcam_turned)

        rate.sleep()