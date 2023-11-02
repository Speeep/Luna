import rospy
from std_msgs.msg import Int32

def callback(data):
    print('Servo Pos is: %s', data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('servo_pos', Int32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
