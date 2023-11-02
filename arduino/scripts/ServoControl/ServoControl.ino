/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

Servo servo;

float p = 0.005;

std_msgs::Int32 servo_pos_msg; // Define the message for servo_pos
ros::Publisher servo_pos_pub("servo_pos", &servo_pos_msg); // Create a publisher for servo_pos

void servo_cb( const std_msgs::Int32& cmd_msg){
  uint32_t error = cmd_msg.data;

  // Read Current Position
  int current_position = servo.read();

  // Calculate what the new position should be
  int new_position = current_position + (error * p);

  if (abs(new_position - current_position) < 1) {
    new_position = current_position;
  }

  // Ensure the new position is within the valid range (0-180)
  if (new_position < 0) {
    new_position = 0;
  } else if (new_position > 180) {
    new_position = 180;
  }

  servo_pos_msg.data = new_position;
  servo_pos_pub.publish(&servo_pos_msg);
  
  Serial.print("Current Position: ");
  Serial.println(current_position);
  Serial.print("New Position: ");
  Serial.println(new_position);
  
  servo.write(new_position); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::Int32> sub("servo_error", servo_cb);

void setup(){

  Serial.begin(9600);
  
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(2); //attach it to pin 2
}

void loop(){
  nh.spinOnce();
  delay(1);
}
