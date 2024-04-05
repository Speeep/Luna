import rospy
from pynput import keyboard

class StateMachine:
    def __init__(self):
        self.states = ['plunging', 'retracting', 'all stop']
        self.current_state = 0
        self.state_names = {
            'f': 0,
            'g': 1,
            'h': 2
        }

        # Create a listener for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Create a loop to continuously print the current state
        rospy.Timer(rospy.Duration(1), self.loop)

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in self.state_names:
                new_state_index = self.state_names[key_char]
                if new_state_index != self.current_state:
                    self.current_state = new_state_index
        except AttributeError:
            pass

    def loop(self):
        print("Current state:", self.states[self.current_state])

if __name__ == '__main__':
    try:
        rospy.init_node('state_machine')
        state_machine = StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
