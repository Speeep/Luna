import rospy
from std_msgs.msg import Int8

def set_wheel_angle(angle):
    wheel_angle_pub = rospy.Publisher('/drivetrain/wheel_angle', Int8, queue_size=10)
    wheel_angle = Int8()
    wheel_angle.data = angle
    wheel_angle_pub.publish(wheel_angle)

def main():
    rospy.init_node('drivetrain', anonymous=True)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        set_wheel_angle(45)
        rate.sleep()

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass