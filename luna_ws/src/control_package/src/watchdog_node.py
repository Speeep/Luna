import rospy
from std_msgs.msg import Bool, Float32, Int32
import subprocess
from time import sleep

FAST_SPEED = 10

# Init last time received
last_time_received = 0
last_fast_speed = 0
recent_setpoint = 0

def watchdog_callback(data):
    global last_time_received
    last_time_received = rospy.get_time()
    rospy.loginfo("Data received from Arduino!")

def setpoint_callback(setpoint):
    global recent_setpoint
    recent_setpoint = setpoint.data

def speed_callback(speed):
    global last_fast_speed, recent_setpoint, last_time_received
    current_time = rospy.get_time()
    
    # If recent setpoint is close to 0, then set last fast speed to now
    if abs(recent_setpoint) < 1000:
        last_fast_speed = current_time
    elif abs(speed.data) > FAST_SPEED:
        last_fast_speed = current_time

    # If long time since last fast time, reset arduino
    if current_time - last_fast_speed > 3:
        rospy.logerr("Weird conveyor speed from arduino, restarting node!")
        subprocess.call(["rosnode", "kill", "/serial_node"])
        subprocess.Popen(["roslaunch", "robot_package", "serial_node.launch"])
        for i in range(32):
            last_fast_speed = rospy.get_time() # Reset to prevent auto retriggers
            last_time_received = rospy.get_time() # Reset to prevent auto retriggers
            sleep(0.25)

def watchdog():
    global last_time_received, last_fast_speed
    rospy.init_node('rosserial_watchdog')
    rospy.Subscriber("/watchdog_bool", Bool, watchdog_callback)
    rospy.Subscriber("/digger/conveyor_current", Int32, setpoint_callback)
    rospy.Subscriber("/jetson/conveyor_speed", Float32, speed_callback)
    rate = rospy.Rate(3)  # check 3 times every second
    sleep(5)
    last_time_received = rospy.get_time()

    while not rospy.is_shutdown():

        rospy.loginfo(rospy.get_time() - last_time_received)

        if rospy.get_time() - last_time_received > 1:  # 1 second without update
            rospy.logerr("No updates from rosserial_python, restarting node!")
            subprocess.call(["rosnode", "kill", "/serial_node"])
            subprocess.Popen(["roslaunch", "robot_package", "serial_node.launch"])
            for i in range(32):
                last_fast_speed = rospy.get_time() # Reset to prevent auto retriggers
                last_time_received = rospy.get_time() # Reset to prevent auto retriggers
                sleep(0.25)
        rate.sleep()

if __name__ == '__main__':
    try:
        watchdog()
    except rospy.ROSInterruptException:
        pass
