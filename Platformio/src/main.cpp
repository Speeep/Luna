#include <Wire.h>
#include <Arduino.h>
#include "./robotMap.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "./peripheral/snowblower.h"
#include "./peripheral/encoder.h"
#include "./peripheral/CANController.h"

ros::NodeHandle nh;

Snowblower left_turn_motor;
Encoder left_wheelpod_encoder;
CANController can_controller;

std_msgs::Float32 left_wheelpod_angle_msg;
ros::Publisher left_wheelpod_angle_pub("/drivetrain/left_wheelpod_angle", &left_wheelpod_angle_msg);

float angleSetpoint = 1;

void angleSetpointCallback(const std_msgs::Float32 &angle_msg)
{
  angleSetpoint = angle_msg.data;
}

ros::Subscriber<std_msgs::Float32> angleSetpointSub("/arduino/left_wheelpod_angle_setpoint", &angleSetpointCallback);

void setup()
{
  Serial.begin(9600);
  SPI.begin();
  Wire.begin();
  Wire.setClock(800000L);

  left_turn_motor.init();
  left_wheelpod_encoder.init(1, 0.0);
  can_controller.init();

  nh.initNode();
  nh.advertise(left_wheelpod_angle_pub);
  nh.subscribe(angleSetpointSub);
}
void loop()
{
  // nh.spinOnce();
  // float angle = left_wheelpod_encoder.getAngle();

  // // Publish the magnetic sensor data to the ROS topic
  // left_wheelpod_angle_msg.data = angle;
  // left_wheelpod_angle_pub.publish(&left_wheelpod_angle_msg);

  // float p = 50;
  // float error = angle - angleSetpoint;

  // // normalize error within -pi to pi
  // if (error > PI) { error -= 2 * PI; }
  // if (error < -PI) { error += 2 * PI; }

  // int effort = (int)(error * p);
  // left_turn_motor.setEffort(0);
  delay(5);

  can_controller.setMotorSpeed(10);
}