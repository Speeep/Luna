import rospy
from pynput import keyboard
from std_msgs.msg import Bool, Int32, Float32
import numpy as np

# Constants
DIGGER_UPDATE_HZ = 20
PLUNGE_BASE_EFFORT = 100
PLUNGE_KP = 6
ZEB_SPEED = 10.24

class StateMachine:
    def __init__(self):
        self.states = ['plunging', 'retracting', 'all stop']
        self.current_state = 2
        self.state_names = {
            'f': 0,
            'g': 1,
            'h': 2
        }

        # Create a listener for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Create a loop to continuously print the current state at 20 Hz
        rospy.Timer(rospy.Duration(1.0 / DIGGER_UPDATE_HZ), self.loop)

        # Initialize publishers for conveyor and plunger
        self.run_conveyor_pub = rospy.Publisher('/digger/run_conveyor', Bool, queue_size=10)
        self.plunge_pub = rospy.Publisher('/digger/plunge', Int32, queue_size=10)

        # Initialize subscriber for conveyor speed
        rospy.Subscriber('/jetson/conveyor_speed', Float32, self.conveyor_speed_callback)

        # Initialize error variable
        self.error = 0

        # Initialize list for storing last 9 speed values
        self.speed_history = []

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in self.state_names:
                new_state_index = self.state_names[key_char]
                if new_state_index != self.current_state:
                    self.current_state = new_state_index
        except AttributeError:
            pass

    def loop(self, event):
        if self.current_state == 0:  # If state is plunging
            # Conveyor publish set effort and monitor speed
            conveyor_run = True
            self.run_conveyor_pub.publish(conveyor_run)

            # Plunger publish plunge speed as a function of conveyor speed
            plunger_speed_base = 30
            plunger_speed = self.calculate_plunge_speed(plunger_speed_base)
            self.plunge_pub.publish(plunger_speed)

        elif self.current_state == 1:  # Else if state is retracting
            # Conveyor set speed to 0.0
            conveyor_run = False
            self.run_conveyor_pub.publish(conveyor_run)

            # Plunger publish constant retract plunge speed
            retract_speed = -50
            self.plunge_pub.publish(retract_speed)

        elif self.current_state == 2:  # Else if state is all stop
            # Stop Conveyor
            conveyor_run = False
            self.run_conveyor_pub.publish(conveyor_run)

            # Stop Plunger
            stop_speed = 0.0
            self.plunge_pub.publish(stop_speed)

    def calculate_plunge_speed(self, base):
        speed = base - (PLUNGE_KP * self.error)
        

    def conveyor_speed_callback(self, msg):

        self.speed_history.append(msg.data)
        
        # If the length of the speed history list exceeds 9, remove the oldest value
        if len(self.speed_history) > 9:
            del self.speed_history[0]
        
        # Apply median filter to the speed history list
        median_speed = np.median(self.speed_history)
        
        # Update error using the median speed value
        self.error = ZEB_SPEED - median_speed

if __name__ == '__main__':
    try:
        rospy.init_node('state_machine')
        state_machine = StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
