import rospy
from std_msgs.msg import Int32
from serial_node import SerialNode

def servo_error_callback(data):
    # Create a message to send to the Arduino
    msg = Int32()
    msg.data = data.data

    # Send the data to the Arduino
    serial_node.send(msg)

if __name__ == '__main__':
    rospy.init_node('servo_error_publisher')
    
    # Create a SerialNode to communicate with the Arduino
    serial_node = SerialNode('/dev/ttyUSB0', 57600)
    
    # Subscribe to the 'servo_error' topic
    rospy.Subscriber('servo_error', Int32, servo_error_callback)
    
    rospy.spin()