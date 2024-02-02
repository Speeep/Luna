import rospy
from std_msgs.msg import Float32MultiArray

pose_step = [0.0, 0.0, 0.0]


def update_odom_data_cb(odom_msg):
    pose_step = odom_msg.data
    print(len(pose_step))
    print(type(pose_step))
    # print(pose_step)

def filter():

    rospy.init_node('localization_filter', anonymous=True)

    rospy.Subscriber('/jetson/pose_step', Float32MultiArray, update_odom_data_cb)

    rospy.spin()

if __name__ == '__main__':
    filter()
