import rospy
from std_msgs.msg import Bool
import subprocess
from time import sleep

def callback(data):
    global last_time_received
    last_time_received = rospy.get_time()
    print(f"Most recent time is: {last_time_received}")

def watchdog():
    rospy.init_node('rosserial_watchdog')
    rospy.Subscriber("/watchdog_bool", Bool, callback)
    rate = rospy.Rate(3)  # check 3 times every second
    sleep(10)
    while not rospy.is_shutdown():
        if rospy.get_time() - last_time_received > 1:  # 1 second without update
            print("No updates from rosserial_python, restarting node...")
            subprocess.call(["rosnode", "kill", "/serial_node"])
            subprocess.call(["roslaunch", "robot_package", "serial_node.launch"])
            sleep(10) # Sleep to let the node relaunch
            last_time_received = rospy.get_time() # Reset time to prevent auto retriggers
        rate.sleep()

if __name__ == '__main__':
    last_time_received = rospy.get_time()
    try:
        watchdog()
    except rospy.ROSInterruptException:
        pass
