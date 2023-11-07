import rospy
from std_msgs.msg import Int32

def callback(data):
    print('Transformation Data is: %s', data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('transformation_data', Int32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
