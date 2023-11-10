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

float p = 0.001;

std_msgs::Int32 servo_speed_msg;
ros::Publisher pub("servo_speed", &servo_speed_msg);

void servo_cb( const std_msgs::Int32& cmd_msg){
  uint32_t error = cmd_msg.data;

  // Calculate what the new position should be
  int speed = 90 + int(error * p);

  // Ensure the new position is within the valid range (0-180)
  if (speed < 0) {
    speed = 0;
  } else if (speed > 180) {
    speed = 180;
  }
  
  servo.write(speed); //set servo angle, should be from 0-180
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::Int32> sub("servo_error", servo_cb);

void setup(){

  Serial.begin(9600);
  
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  
  servo.attach(2); //attach it to pin 2
}

void loop(){
  nh.spinOnce();
  delay(1);
}
