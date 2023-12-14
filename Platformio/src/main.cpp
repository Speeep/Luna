#include <Wire.h>
#include <Arduino.h>
#include "./robotMap.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include "./subsystems/drivetrain.h"

ros::NodeHandle nh;

std_msgs::Float32 left_wheelpod_angle_msg;
ros::Publisher left_wheelpod_angle_pub("/listener/left_wheelpod_angle", &left_wheelpod_angle_msg);

std_msgs::Float32 left_wheelpod_angle_setpoint_msg;
ros::Publisher left_wheelpod_angle_setpoint_pub("/listener/left_wheelpod_angle_setpoint", &left_wheelpod_angle_setpoint_msg);

std_msgs::Int32 motorSpeed;
ros::Publisher motorSpeedPub("/listener/motorspeed", &motorSpeed);

Drivetrain drivetrain;

int driveSpeed = 0;
bool drivetrainEnable = false;
bool drivetrainAngle = false;

void driveSpeedCallback(const std_msgs::Int32 &driveSpeedMsg) {
  driveSpeed = driveSpeedMsg.data;
}

void leftWheelpodAngleCallback(const std_msgs::Float32 &leftWheelpodAngleMsg) {
  drivetrain.setLeftWheelpodAngleSetpoint(leftWheelpodAngleMsg.data);
}

void drivetrainEnableCallback(const std_msgs::Bool &driveEnableMsg) {
  drivetrainEnable = driveEnableMsg.data;

  if (drivetrainEnable) {
    drivetrain.enable();
  } else {
    drivetrain.disable();
  }
}

void drivetrainAngleCallback(const std_msgs::Bool &driveAngleMsg) {
  drivetrainAngle = driveAngleMsg.data;

  drivetrain.setAngle(drivetrainAngle);

}

ros::Subscriber<std_msgs::Int32> driveSpeedSub("/drivetrain/drive", &driveSpeedCallback);
ros::Subscriber<std_msgs::Float32> leftWheelpodAngleSub("/drivetrain/left_wheelpod_angle_setpoint", &leftWheelpodAngleCallback);
ros::Subscriber<std_msgs::Bool> driveEnableSub("/drivetrain/enable", &drivetrainEnableCallback);
ros::Subscriber<std_msgs::Bool> driveAngleSub("/drivetrain/angle", &drivetrainAngleCallback);

void setup()
{
  Serial.begin(57600);
  SPI.begin();
  Wire.begin();
  Wire.setClock(800000L);

  drivetrain.init();

  nh.initNode();
  nh.advertise(left_wheelpod_angle_pub);
  nh.advertise(motorSpeedPub);
  nh.subscribe(driveSpeedSub);
  nh.subscribe(driveEnableSub);
}
void loop()
{
  nh.spinOnce();

  // drivetrain.setWheelSpeeds(300, 300, 300, 300);

  // int motor4speed = drivetrain.getSpeed(0);
  // motorSpeed.data = motor4speed;
  // motorSpeedPub.publish(&motorSpeed);

  float left_wheelpod_angle = drivetrain.getLeftWheelpodAngle();
  left_wheelpod_angle_msg.data = left_wheelpod_angle;
  left_wheelpod_angle_pub.publish(&left_wheelpod_angle_msg);

  float left_wheelpod_angle_setpoint = drivetrain.getLeftWheelpodAngleSetpoint();
  left_wheelpod_angle_setpoint_msg.data = left_wheelpod_angle_setpoint;
  left_wheelpod_angle_setpoint_pub.publish(&left_wheelpod_angle_setpoint_msg);

  drivetrain.loop();

  // drivetrain.setWheelSpeeds(0, 0, 0, 0);
}