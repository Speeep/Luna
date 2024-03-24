import rospy
from std_msgs.msg import Float32, Bool, Int32
from pynput import keyboard

class KeyControlNode:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)

        self.icc = 0

        self.conveyer_running = False

        # Define publishers for different key presses
        self.drivetrain_drive_pub = rospy.Publisher('/drivetrain/drive', Float32, queue_size=10)
        self.drivetrain_state_pub = rospy.Publisher('/drivetrain/state', Int32, queue_size=10)
        self.drivetrain_icc_step_pub = rospy.Publisher('/drivetrain/icc_step', Float32, queue_size=10)
        self.drivetrain_icc_pub = rospy.Publisher('/drivetrain/icc', Float32, queue_size=10)
        self.localizer_error_pub = rospy.Publisher('localizer/error', Float32, queue_size=10)
        self.localizer_enable_pub = rospy.Publisher('/localizer/enable', Bool, queue_size=10)
        self.run_conveyer_pub = rospy.Publisher('/digger/run_conveyer', Bool, queue_size=10)
        self.plunge_pub = rospy.Publisher('/digger/plunge', Float32, queue_size=10)

        # Create a listener for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Initialize variables to track key states
        self.key_states = {
            'w': False,
            's': False,
            'q': False,
            'e': False,
            'n': False,
            'm': False,
            'b': False,
            '0': False,
            '1': False,
            '2': False,
            '3': False,
            'z': False,
            'a': False,
            'd': False,
        }

        # Create a timer to check key presses periodically
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_key_presses)

        # Robot enable
        self.drivetrain_state = 0

        # Localizer Enable
        self.localizer_enable = False

        # Localizer Angle Setpoint
        self.localizer_error = 0.0

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in self.key_states:
                self.key_states[key_char] = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            key_char = key.char
            if key_char in self.key_states:
                self.key_states[key_char] = False
        except AttributeError:
            pass

    def check_key_presses(self, event):
        # Keys needed for driving forward and backward
        if self.key_states['w']:
            drive_speed = Float32()
            drive_speed.data = 0.6
            self.drivetrain_drive_pub.publish(drive_speed)
        elif self.key_states['s']:
            drive_speed = Float32()
            drive_speed.data = -0.6
            self.drivetrain_drive_pub.publish(drive_speed)
        else:
            drive_speed = Float32()
            drive_speed.data = 0.0
            self.drivetrain_drive_pub.publish(drive_speed)
            
        # Keys needed for moving the ICC
        if self.key_states['q']:
            ICC_step = Float32()
            ICC_step.data = 0.02
            self.icc += .02
            ICC = Float32()
            ICC.data = self.icc
            self.drivetrain_icc_step_pub.publish(ICC_step)
            self.drivetrain_icc_pub.publish(ICC)
        elif self.key_states['e']:
            ICC_step = Float32()
            ICC_step.data = -0.02
            self.icc -= .02
            ICC = Float32()
            ICC.data = self.icc
            self.drivetrain_icc_step_pub.publish(ICC_step)
            self.drivetrain_icc_pub.publish(ICC)
        else:
            ICC_step = Float32()
            ICC_step.data = 0.0
            self.drivetrain_icc_step_pub.publish(ICC_step)

        # Keys needed for enabling and disabling the robot
            
        state = Int32()
        localizer_enable = Bool()

        if self.key_states['0']:
            self.drivetrain_state = 0
            self.localizer_enable = False
        elif self.key_states['1']:
            self.drivetrain_state = 1
            self.localizer_enable = True
            localizer_enable.data = self.localizer_enable
        elif self.key_states['2']:
            self.drivetrain_state = 2
            self.localizer_enable = True
            localizer_enable.data = self.localizer_enable
        elif self.key_states['3']:
            self.drivetrain_state = 3
            self.localizer_enable = True
            localizer_enable.data = self.localizer_enable

        state.data = self.drivetrain_state
        self.drivetrain_state_pub.publish(state)
        localizer_enable.data = self.localizer_enable
        self.localizer_enable_pub.publish(localizer_enable)

        if self.key_states['b']:
            self.localizer_error = 100.0
            self.localizer_error_pub.publish(self.localizer_error)
        elif self.key_states['m']:
            self.localizer_error = -100.0
            self.localizer_error_pub.publish(self.localizer_error)
        elif self.key_states['n']:
            self.localizer_error = 0.0
            self.localizer_error_pub.publish(self.localizer_error)

        if self.key_states['z']:
            self.conveyer_running = not self.conveyer_running
            self.run_conveyer_pub.publish(self.conveyer_running)
        
        if self.key_states['a']:
            self.plunge_pub.publish(1.0)
        elif self.key_states['d']:
            self.plunge_pub.publish(-1.0)
        else:
            self.plunge_pub.publish(0)

if __name__ == '__main__':
    try:
        key_control_node = KeyControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass