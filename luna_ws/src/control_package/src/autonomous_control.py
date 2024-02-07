import rospy
from std_msgs.msg import Float32, Bool
from pynput import keyboard


# Robot variables
localizer_error = 0.0
localizer_enable = False
drivetrain_enable = False
localizer_angle = 0.0
drivetrain_angle = False


def update_localizer_error_cb(localizer_error_msg):
    global localizer_error
    localizer_error = localizer_error_msg.data

def update_localizer_enable_cb(localizer_enable_msg):
    global localizer_enable
    localizer_enable = localizer_enable_msg.data

def update_drivetrain_enable_cb(drivetrain_enable_msg):
    global drivetrain_enable
    drivetrain_enable = drivetrain_enable_msg.data

def update_localizer_angle_cb(localizer_angle_msg):
    global localizer_angle
    localizer_angle = localizer_angle_msg.data

def update_drivetrain_angle_cb(drivetrain_angle_msg):
    global drivetrain_angle
    drivetrain_angle = drivetrain_angle_msg.data

def main():
    global localizer_error, localizer_enable, drivetrain_enable, localizer_angle
    rospy.init_node('keyboard_control', anonymous=True)

    # Define publishers for different key presses
    drivetrain_drive_pub = rospy.Publisher('/drivetrain/drive', Float32, queue_size=10)
    drivetrain_angle_pub = rospy.Publisher('/drivetrain/angle', Bool, queue_size=10)
    drivetrain_rotate_pub = rospy.Publisher('/drivetrain/rotate', Float32, queue_size=10)
    localizer_error_pub = rospy.Publisher('/localizer/error', Float32, queue_size=10)

    # Subscribers for Autonomous Routine
    rospy.Subscriber('/localizer/error', Float32, update_localizer_error_cb)
    rospy.Subscriber('/localizer/enable', Bool, update_localizer_enable_cb)
    rospy.Subscriber('/drivetrain/enable', Bool, update_drivetrain_enable_cb)
    rospy.Subscriber('/jetson/localizer_angle', Float32, update_localizer_angle_cb)
    rospy.Subscriber('/drivetrain/angle', Bool, update_drivetrain_angle_cb)


    rate = rospy.Rate(5)

    while not rospy.is_shutdown():

        # Autonomous Code Here

        # Only if the localizer is enabled
        if (localizer_enable):

            # Use localizer to search for the marker
            if (localizer_error == 0.0):
                localizer_error = 500.0
            elif (localizer_error == 500.0):
                localizer_error = 0.0
            localizer_error_pub.publish(localizer_error)
        else: 
            localizer_error_pub.publish(0.0)

        # Only if the robot is enabled
        if (drivetrain_enable):

            print("robot is enabled!")

            # If localizer can see the ArUco marker
            if (localizer_error != 0.0 and abs(localizer_error) < 150):

                print("localizer can see the ArUco marker!")

                # If robot is facing the left of the aruco marker
                if (localizer_angle > 0.1):

                    # If the drivetrain is not angled, set wheelpod angle setpoint to 45 degrees
                    if (not drivetrain_angle):
                        drivetrain_angle_pub.publish(True)

                    drivetrain_drive_pub.publish(-0.4)

                    print("turning right!")

                # If robot is facing the right of the aruco marker
                elif (localizer_angle < -0.1):

                    # If the drivetrain is not angled, set wheelpod angle setpoint to 45 degrees
                    if (not drivetrain_angle):
                        drivetrain_angle_pub.publish(True)

                    drivetrain_drive_pub.publish(0.4)

                    print("turning left!")

                else:
                    # If the drivetrain is not angled, set wheelpod angle setpoint to 45 degrees
                    if (drivetrain_angle):
                        drivetrain_angle_pub.publish(False)

                    drivetrain_drive_pub.publish(0.4)

        else: 
            drivetrain_drive_pub.publish(0.0)

                    


        rate.sleep()

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass