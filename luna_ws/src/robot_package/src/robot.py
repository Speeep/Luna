# Import necessary libraries
import rospy


def main():
    rospy.init_node('robot')
    
    rospy.spin()

if __name__ == '__main__':
    print("I am a robot!")
    main()