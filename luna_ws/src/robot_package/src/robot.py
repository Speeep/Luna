# Import necessary libraries
import rospy
from std_msgs.msg import Bool


# Keyboard control flag
KEYBOARD_CONTROL = True

# Flag to define if the robot is enabled or diabled
robot_enable = False

def publish_robot_enable(bool):

    # All Subsystem Publishers go here
    drivetrain_enable_pub = rospy.Publisher('/drivetrain/enable', Bool, queue_size=10)

    # Message to be Published
    enable = Bool()
    enable.data = bool

    # Publish Message to each Subsystem Here
    drivetrain_enable_pub.publish(enable)

# Function to initialize and enable all robot subsystems
def initialize_subsystems():
    global robot_enable
    robot_enable = True

def main():

    # Initialize the ros node
    rospy.init_node('robot')

    # Publish at 10 Hz
    rate = rospy.Rate(1)

    initialize_subsystems()

    # Code gets looped here
    while not rospy.is_shutdown():
        
        if not KEYBOARD_CONTROL:
            # Publish to all robot systems if robot is enabled or disabled
            publish_robot_enable(robot_enable)

    rate.sleep()
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    print("I am a robot!")
    main()