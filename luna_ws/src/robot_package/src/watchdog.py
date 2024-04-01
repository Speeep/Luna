#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO
import time

class Watchdog:
    def __init__(self):
        rospy.init_node('arduino_node', anonymous=True)

        # Set up GPIO pin
        self.reset_pin = 2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.reset_pin, GPIO.OUT)

        # Initialize timer variables
        self.last_message_time = time.time()

        # Subscribe to the Arduino topic
        rospy.Subscriber("/watchdog", Int32, self.watchdog_cb)

    def watchdog_cb(self, data):
        # Update the last message time
        self.last_message_time = time.time()

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Check if timeout has occurred
            if time.time() - self.last_message_time > 5:
                # Reset Arduino by setting GPIO pin LOW
                GPIO.output(self.reset_pin, GPIO.LOW)
                rospy.loginfo("Arduino reset triggered")
                time.sleep(0.5)  # Ensure reset signal is sent
                # Set GPIO pin back to HIGH
                GPIO.output(self.reset_pin, GPIO.HIGH)

            # Set GPIO pin 2 to HIGH constantly
            GPIO.output(self.reset_pin, GPIO.HIGH)

            rate.sleep()

if __name__ == '__main__':
    try:
        arduino_node = Watchdog()
        arduino_node.run()
    except rospy.ROSInterruptException:
        pass