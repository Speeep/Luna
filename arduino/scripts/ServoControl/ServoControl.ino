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

#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

Servo servo;

int speed = 0;
int outSpeed;

float p = -0.04;

std_msgs::Int32 servo_speed_msg;
ros::Publisher pub("servo_speed", &servo_speed_msg);

void servo_cb( const std_msgs::Int32& cmd_msg){
  int32_t error = cmd_msg.data;

  speed = int(p * error);
  servo_speed_msg.data = speed;

  // Publish the message and check for success
  if (pub.publish(&servo_speed_msg)) {
    nh.spinOnce();
  } else {
    Serial.println("Failed to publish servo_speed_msg");
  }
}


ros::Subscriber<std_msgs::Int32> sub("servo_error", servo_cb);

void setup(){

  Serial.begin(9600);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  
  servo.attach(2);
  servo.write(91);
}

void loop(){
  nh.spinOnce();

  //Speed Vals can be from -10 to +10

  if (speed < 0) {
    outSpeed = map(speed, -10, 0, 85, 86);
  }
  else if (speed > 0) {
    outSpeed = map(speed, 0, 10, 92, 94);
  }
  else {
    outSpeed = 91;
  }

  servo.write(outSpeed);
  
  delay(1);
}
