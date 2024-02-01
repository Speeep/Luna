import rospy
from geometry_msgs.msg import Float32MultiArray

def callback(data):
    print(data.data)

def listener():

    rospy.init_node('localization_filter', anonymous=True)

    rospy.Subscriber('/jetson/pose', Float32MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
