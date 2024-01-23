import rospy
from std_msgs.msg import Float32, Bool, Int32
from pynput import keyboard

class KeyControlNode:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)

        # Define publishers for different key presses
        self.drivetrain_drive_pub = rospy.Publisher('/drivetrain/drive', Float32, queue_size=10)
        self.drivetrain_angle_pub = rospy.Publisher('/drivetrain/angle', Bool, queue_size=10)
        self.drivetrain_rotate_pub = rospy.Publisher('/drivetrain/rotate', Float32, queue_size=10)
        self.drivetrain_enable_pub = rospy.Publisher('/drivetrain/enable', Bool, queue_size=10)

        # Create a listener for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Initialize variables to track key states
        self.key_states = {
            'w': False,
            's': False,
            'q': False,
            'e': False,
            'a': False,
            'd': False,
            'o': False,
            'p': False
        }

        # Create a timer to check key presses periodically
        self.timer = rospy.Timer(rospy.Duration(0.01), self.check_key_presses)

        # Robot enable
        self.robot_enable = False

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

        # Keys needed for angling the wheel pods in and out
        if self.key_states['q']:
            angled = Bool()
            angled.data = True
            self.drivetrain_angle_pub.publish(angled)
        elif self.key_states['e']:
            angled = Bool()
            angled.data = False
            self.drivetrain_angle_pub.publish(angled)

        # Keys needed for rotating the drivetrain about its center axis
        if self.key_states['a']:
            rotate_speed = Float32()
            rotate_speed.data = 0.4
            self.drivetrain_rotate_pub.publish(rotate_speed)
        elif self.key_states['d']:
            rotate_speed = Float32()
            rotate_speed.data = -0.4
            self.drivetrain_rotate_pub.publish(rotate_speed)
        else:
            rotate_speed = Float32()
            rotate_speed.data = 0.0
            self.drivetrain_rotate_pub.publish(rotate_speed)

        # Keys needed for enabling and disabling the robot
        if self.key_states['p']:
            disable = Bool()
            self.robot_enable = False
            disable.data = self.robot_enable
            self.drivetrain_enable_pub.publish(self.robot_enable)
        elif self.key_states['o']:
            enable = Bool()
            self.robot_enable = True
            enable.data = self.robot_enable
            self.drivetrain_enable_pub.publish(self.robot_enable)
        else:
            disable = Bool()
            disable.data = self.robot_enable
            self.drivetrain_enable_pub.publish(self.robot_enable)

if __name__ == '__main__':
    try:
        key_control_node = KeyControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass