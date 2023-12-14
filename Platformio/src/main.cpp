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
ros::Publisher left_wheelpod_angle_pub("/drivetrain/left_wheelpod_angle", &left_wheelpod_angle_msg);

std_msgs::Int32 motorSpeed;
ros::Publisher motorSpeedPub("/motorspeed", &motorSpeed);

Drivetrain drivetrain;

int driveSpeed = 0;

bool drivetrainEnable = false;

void driveSpeedCallback(const std_msgs::Int32 &driveSpeedMsg)
{
  driveSpeed = driveSpeedMsg.data;
}

void drivetrainEnableCallback(const std_msgs::Bool &driveEnableMsg)
{
  drivetrainEnable = driveEnableMsg.data;

  if (drivetrainEnable) {
    drivetrain.enable();
  } else {
    drivetrain.disable();
  }
}

ros::Subscriber<std_msgs::Int32> driveSpeedSub("/drivetrain/drive", &driveSpeedCallback);

ros::Subscriber<std_msgs::Bool> driveEnableSub("/drivetrain/enable", &drivetrainEnableCallback);

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

  drivetrain.setWheelSpeeds(300, 300, 300, 300);

  int motor4speed = drivetrain.getSpeed(0);
  motorSpeed.data = motor4speed;
  motorSpeedPub.publish(&motorSpeed);

  // drivetrain.setWheelSpeeds(0, 0, 0, 0);
}