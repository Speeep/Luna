import rospy
from std_msgs.msg import Int32, Float32

def callback(data):
    print("Left Wheelpod Angle Setpoint: " + str(data.data))

def anglecallback(data):
    print("Left Wheelpod Angle: " + str(data.data))

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/drivetrain/left_wheelpod_angle_setpoint", Float32, callback)

    rospy.Subscriber('/drivetrain/left_wheelpod_angle', Float32, anglecallback)

    rospy.spin()

if __name__ == '__main__':
    listener()