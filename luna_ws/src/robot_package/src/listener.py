import rospy
from std_msgs.msg import Int8

def callback(data):
    pass

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/drivetrain/wheel_angle', Int8, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()