import rospy
from std_msgs.msg import Float32, Bool
from pynput import keyboard

class AutoControlNode:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)

        # Define publishers for different key presses
        self.drivetrain_drive_pub = rospy.Publisher('/drivetrain/drive', Float32, queue_size=10)
        self.drivetrain_angle_pub = rospy.Publisher('/drivetrain/angle', Bool, queue_size=10)
        self.drivetrain_rotate_pub = rospy.Publisher('/drivetrain/rotate', Float32, queue_size=10)
        self.drivetrain_enable_pub = rospy.Publisher('/drivetrain/enable', Bool, queue_size=10)
        self.localizer_error_pub = rospy.Publisher('localizer/error', Float32, queue_size=10)
        self.localizer_enable_pub = rospy.Publisher('/localizer/enable', Bool, queue_size=10)

        # Create a listener for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Initialize variables to track key states
        self.key_states = {
            'o': False,
            'p': False,
        }

        # Create a timer to check key presses periodically
        self.timer = rospy.Timer(rospy.Duration(0.01), self.check_key_presses)

        # Robot variables
        self.robot_enable = False
        self.robot_has_localizer = False


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

        # Keys needed for enabling and disabling the robot
        if self.key_states['p']:
            disable = Bool()
            self.robot_enable = False
            disable.data = self.robot_enable
            self.drivetrain_enable_pub.publish(self.robot_enable)
            self.localizer_enable_pub.publish(self.robot_enable)
        elif self.key_states['o']:
            enable = Bool()
            self.robot_enable = True
            enable.data = self.robot_enable
            self.drivetrain_enable_pub.publish(self.robot_enable)
            self.localizer_enable_pub.publish(self.robot_enable)
        else:
            disable = Bool()
            disable.data = self.robot_enable
            self.drivetrain_enable_pub.publish(self.robot_enable)
            self.localizer_enable_pub.publish(self.robot_enable)


if __name__ == '__main__':
    try:
        auto_control_node = AutoControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass