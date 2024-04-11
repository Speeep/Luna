import rospy
from pynput import keyboard
from std_msgs.msg import Int32, Float32
import numpy as np

# Constants
DIGGER_UPDATE_HZ = 20
PLUNGE_BASE_EFFORT = 100
PLUNGE_KP = 10
ZEB_SPEED = 10.24
UNJAM_ERROR = 9
UNJAM_DURATION = 0.5
MIN_SPEED_READINGS = 100
MIN_PLUNGE_READINGS = 200

class StateMachine:
    def __init__(self):
        self.states = ['plunging', 'retracting', 'all stop']
        self.current_state = 2
        self.state_names = {
            'f': 0,
            'g': 1,
            'h': 2
        }

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Create a loop to continuously print the current state at 20 Hz
        rospy.Timer(rospy.Duration(1.0 / DIGGER_UPDATE_HZ), self.loop)

        # Initialize publishers for conveyor and plunger
        self.run_conveyor_pub = rospy.Publisher('/digger/conveyor_current', Int32, queue_size=10)
        self.plunge_pub = rospy.Publisher('/digger/plunge', Int32, queue_size=10)

        # Initialize subscriber for conveyor speed
        rospy.Subscriber('/jetson/conveyor_speed', Float32, self.conveyor_speed_callback)

        # Initialize error variable
        self.error = 0

        # Initialize list for storing last 9 speed values
        self.speed_history = []

        # Jam Timer
        self.jam_time = None
        self.num_speed_readings = 0

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in self.state_names:
                new_state_index = self.state_names[key_char]
                if new_state_index != self.current_state:
                    self.current_state = new_state_index
                    self.num_speed_readings = 0
                    self.speed_history = []
        except AttributeError:
            pass

    def loop(self, event):

        # Get current ROS time
        current_time = rospy.get_time()

        # Check for All Stop case first for safety reasons
        if self.current_state == 2:
            # Stop Conveyor
            conveyor_run = False
            self.run_conveyor_pub.publish(0)

            # Stop Plunger
            stop_speed = 0
            self.plunge_pub.publish(stop_speed)
            self.jam_time = None
        
        else:

            # Block other stuff from happening
            if self.jam_time is not None and (current_time - self.jam_time) < UNJAM_DURATION:
                self.num_speed_readings = 0
                return

            if self.current_state == 0:  # If state is plunging
                # Conveyor publish set effort and monitor speed
                conveyor_run = True
                self.run_conveyor_pub.publish(10000)

                # Plunger publish plunge speed as a function of conveyor speed
                plunger_speed_base = 10
                plunger_speed = self.calculate_plunge_speed(plunger_speed_base)

                if self.num_speed_readings > MIN_PLUNGE_READINGS:
                    self.plunge_pub.publish(plunger_speed)
                else:
                    self.plunge_pub.publish(-35)

                # Unjamming logic
                if self.error > UNJAM_ERROR and self.num_speed_readings > MIN_SPEED_READINGS:
                    self.run_conveyor_pub.publish(-10000)
                    self.plunge_pub.publish(-35)
                    self.num_speed_readings = 0
                    self.jam_time = current_time  # Record the jam start time

            elif self.current_state == 1:  # Else if state is retracting
                # Conveyor set speed to 0.0
                conveyor_run = False
                self.run_conveyor_pub.publish(conveyor_run)

                # Plunger publish constant retract plunge speed
                retract_speed = -25
                self.plunge_pub.publish(retract_speed)
                self.jam_time = None

    def calculate_plunge_speed(self, base):
        base = int(base)
        speed = int(base - (PLUNGE_KP * self.error))
        # Ensure that the plunger doesn't go faster than base speed
        if speed > base:
            return base
        # Ensure that the plunger doesn't go faster than negative base speed
        elif speed < 0:
            return 0
        return speed

    def conveyor_speed_callback(self, msg):

        self.speed_history.append(msg.data)
        self.num_speed_readings += 1
        
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
