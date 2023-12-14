import rospy
from std_msgs.msg import Int32, Bool, Float32


# Constant to define the angle in which the drivetrain is acceptably straight
WHEELPOD_ANGLE_THRESHOLD = 3.0

drivetrain_enabled = False
drive_speed = 0
angled = False
rotate_speed = 0
left_wheelpod_angle = 0
left_wheelpod_angle_setpoint = 0
right_wheelpod_angle = 0

def toggle_drivetrain_enable(data):
    global drivetrain_enabled
    drivetrain_enabled = data.data

def set_drive_speed(data):
    global drive_speed
    drive_speed = 0
    if (angled == False 
    and abs(left_wheelpod_angle) < WHEELPOD_ANGLE_THRESHOLD 
    and abs(right_wheelpod_angle) < WHEELPOD_ANGLE_THRESHOLD):
        drive_speed = data.data

# Function to publish the drivetrain wheel angle setpoint to the arduino
def set_wheel_angle(angle):
    angle_setpoint_pub = rospy.Publisher('/arduino/left_wheelpod_angle_setpoint', Float32, queue_size=10)
    wheel_angle = Float32()
    wheel_angle.data = angle
    angle_setpoint_pub.publish(wheel_angle)

def toggle_wheelpod_angle(data):
    global angled, left_wheelpod_angle, left_wheelpod_angle_setpoint
    angled = data.data
    if angled:
        left_wheelpod_angle_setpoint = 0.785398
        set_wheel_angle(left_wheelpod_angle_setpoint)
    else:
        left_wheelpod_angle_setpoint = 0.0
        set_wheel_angle(left_wheelpod_angle_setpoint)

def set_rotate_speed(data):
    global rotate_speed
    rotate_speed = 0
    if (angled == True 
    and abs(45 - left_wheelpod_angle) < WHEELPOD_ANGLE_THRESHOLD 
    and abs(45 - right_wheelpod_angle) < WHEELPOD_ANGLE_THRESHOLD):
        rotate_speed = data.data

def left_wheelpod_angle_cb(data):
    global left_wheelpod_angle
    left_wheelpod_angle = data.data

def main():

    # Initialize the ros node
    rospy.init_node('drivetrain')
    rospy.Subscriber('/drivetrain/enable', Bool, toggle_drivetrain_enable)
    rospy.Subscriber('/drivetrain/drive', Int32, set_drive_speed)
    rospy.Subscriber('/drivetrain/angle', Bool, toggle_wheelpod_angle)
    rospy.Subscriber('/drivetrain/rotate', Float32, set_rotate_speed)
    rospy.Subscriber('/drivetrain/left_wheelpod_angle', Float32, left_wheelpod_angle_cb)
    
    # Publish at 10 Hz
    rate = rospy.Rate(10)

    # Code gets looped here
    while not rospy.is_shutdown():

        # Put drivetrain related code here
        if drivetrain_enabled:

            print('\t')
            print(f'Left wheel angle is: {left_wheelpod_angle} radians')
            print(f'Left wheel angle setpoint is: {left_wheelpod_angle_setpoint} radians')
            print(f'Drive speed is: {drive_speed}')
            print(f'Rotating speed is: {rotate_speed}')
            print(f'The wheel pods are angled: {angled}')

        else: 
            print('Drivetrain is not enabled..')

        rate.sleep()

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass