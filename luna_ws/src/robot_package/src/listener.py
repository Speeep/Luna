import rospy
from std_msgs.msg import Int32

def callback(data):
    print("Motor speed: " + str(data.data))

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/motorspeed', Int32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()