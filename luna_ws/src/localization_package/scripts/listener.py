import rospy
from std_msgs.msg import Float32

def callback(data):
    print(f'Cam angle is: {data.data}')

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('magnetic_pos', Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

