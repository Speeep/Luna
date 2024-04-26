import rospy
from std_msgs.msg import Bool
import subprocess
from time import sleep

# Init last time received
last_time_received = 0

def callback(data):
    global last_time_received
    last_time_received = rospy.get_time()
    print(f"Most recent time is: {last_time_received}")

def watchdog():
    global last_time_received
    rospy.init_node('rosserial_watchdog')
    rospy.Subscriber("/watchdog_bool", Bool, callback)
    rate = rospy.Rate(3)  # check 3 times every second
    wait_rate = rospy.Rate(0.2)
    sleep(5)
    last_time_received = rospy.get_time()

    while not rospy.is_shutdown():

        print(rospy.get_time() - last_time_received)

        if rospy.get_time() - last_time_received > 1:  # 1 second without update
            print("No updates from rosserial_python, restarting node...")
            subprocess.call(["rosnode", "kill", "/serial_node"])
            subprocess.call(["roslaunch", "robot_package", "serial_node.launch"])
            wait_rate.sleep() # Sleep to let the node relaunch
            last_time_received = rospy.get_time() # Reset time to prevent auto retriggers
        rate.sleep()

if __name__ == '__main__':
    try:
        watchdog()
    except rospy.ROSInterruptException:
        pass
