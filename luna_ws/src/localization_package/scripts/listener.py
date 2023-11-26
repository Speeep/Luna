import rospy
from std_msgs.msg import Int32

def callback(data):
    print(f'Servo speed is: {data.data}')

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('servo_speed', Int32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
