import rospy
from std_msgs.msg import Int32, Float32, Bool, String

def callback(data):
    print("Left Wheelpod Angle Setpoint: " + str(data.data))

def anglecallback(data):
    print("Left Wheelpod Angle: " + str(data.data))

def enabledcallback(data):
    enabled = data.data

    if enabled:
        print("The drivetrain is ENABLED AND READY TO DRIVE")
    else:
        print("The drivetrain is NOT ENABLED")

def ianoutputcallback(data):
     print(data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("/listener/left_wheelpod_angle_setpoint", Float32, callback)

    # rospy.Subscriber("/listener/left_wheelpod_angle", Float32, anglecallback)

    # rospy.Subscriber("/listener/drivetrain_enabled", Bool, enabledcallback)

    rospy.Subscriber("/listener/ian_output", String, ianoutputcallback)

    rospy.spin()

if __name__ == '__main__':
    listener()