import rospy

def drivetrain_callback(msg):
    print("I am a drivetrain!")

def main():
    rospy.init_node('drivetrain', anonymous=True)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()