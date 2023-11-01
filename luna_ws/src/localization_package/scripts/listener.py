import rospy
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Servo Error is: %s', data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('servo_error', Int32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
