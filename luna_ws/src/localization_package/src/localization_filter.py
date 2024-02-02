import rospy
from std_msgs.msg import Float32MultiArray

# Constants for filter tuning
# alpha = 0.2  # Weight for localization estimates # TODO Un comment this line after testing
# beta = 0.8   # Weight for pose steps # TODO Un comment this line after testing

alpha = 0.0  # Weight for localization estimates # TODO Delete this line after testing
beta = 1.0   # Weight for pose steps # TODO Delete this line after testing

pose = (0.0, 0.0, 0.0)
pose_step = (0.0, 0.0, 0.0)
localization_estimate = (0.0, 0.0, 0.0)
odom_timeout = 3.0
localizer_timeout = 10000.0 # TODO Delete this line after testing
# localizer_timeout = 10.0 # TODO Un comment this line after testing
last_odom_time = rospy.Time(0)
last_localization_time = rospy.Time(0)


def update_odom_data_cb(odom_msg):
    global pose_step, last_odom_time
    pose_step = odom_msg.data
    last_odom_time = rospy.Time.now()

def update_localization_estimate_cb(localization_estimate_msg):
    global localization_estimate, last_localization_time
    localization_estimate = localization_estimate_msg.data
    last_localization_time = rospy.Time.now()

def filter():
    
    global pose

    rospy.init_node('localization_filter', anonymous=True)

    last_odom_time = rospy.Time.now()
    last_localization_time = rospy.Time.now()

    rospy.Subscriber('/jetson/pose_step', Float32MultiArray, update_odom_data_cb)

    rospy.Subscriber('jetson/localization_estimate', Float32MultiArray, update_localization_estimate_cb)

    filtered_pose_pub = rospy.Publisher('jetson/filtered_pose', Float32MultiArray, queue_size=10)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        # If there isn't a base pose yet, use 100% localization estimate from webcam
        if pose == (0.0, 0.0, 0.0):

            # If localization_estimate is also (0.0, 0.0, 0.0) this will result in an infinite loop until timeout.
            pose = localization_estimate

        print(current_time.to_sec())
        print(last_odom_time.to_sec())
        print(current_time - last_odom_time)
        print((current_time - last_odom_time).to_sec())
        print((current_time - last_odom_time).to_sec() > odom_timeout)

        # Check for timeouts
        if (current_time - last_odom_time).to_sec() > odom_timeout or (current_time - last_localization_time).to_sec() > localizer_timeout:
            rospy.logerr("Timeout occurred!")
            pose = (0.0, 0.0, 0.0)

        else:

            # Complementary Filter Here
            fused_pose = (
                alpha * localization_estimate[0] + beta * (pose[0] + pose_step[0]),
                alpha * localization_estimate[1] + beta * (pose[1] + pose_step[1]),
                alpha * localization_estimate[2] + beta * (pose[2] + pose_step[2])
            )

            # Update the pose
            pose = fused_pose

            filtered_pose_msg = Float32MultiArray(data=fused_pose)
            filtered_pose_pub.publish(filtered_pose_msg)
            
            print(pose)
        
        rate.sleep()  

if __name__ == '__main__':
    filter()
