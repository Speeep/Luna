#include <Wire.h>
#include <Arduino.h>
#include "./robotMap.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "./subsystems/drivetrain.h"

ros::NodeHandle nh;

std_msgs::Float32 left_wheelpod_angle_msg;
ros::Publisher left_wheelpod_angle_pub("/listener/left_wheelpod_angle", &left_wheelpod_angle_msg);

std_msgs::Float32 left_wheelpod_angle_setpoint_msg;
ros::Publisher left_wheelpod_angle_setpoint_pub("/listener/left_wheelpod_angle_setpoint", &left_wheelpod_angle_setpoint_msg);

std_msgs::Bool enabbledMsg;
ros::Publisher drivetrainIsEnabledPub("/listener/drivetrain_enabled", &enabbledMsg);

std_msgs::String ianOutputMsg;
ros::Publisher ianOutputPub("/listener/ian_output", &ianOutputMsg);

std_msgs::Int32 motorSpeed;
ros::Publisher motorSpeedPub("/listener/motorspeed", &motorSpeed);

Drivetrain drivetrain;

int driveSpeed = 0;
bool drivetrainEnable = false;
bool drivetrainAngle = false;

void drivetrainSpeedCallback(const std_msgs::Int32 &driveSpeedMsg) {
  drivetrain.setDriveSpeed(driveSpeedMsg.data);
}

// void drivetrainEnableCallback(const std_msgs::Bool &driveEnableMsg) {
//   drivetrainEnable = driveEnableMsg.data;

//   if (drivetrainEnable == true) {
//     drivetrain.enable();
//   } else {
//     drivetrain.disable();
//   }
// }

void drivetrainAngleCallback(const std_msgs::Bool &driveAngleMsg) {
  drivetrainAngle = driveAngleMsg.data;

  drivetrain.setAngle(drivetrainAngle);

}

ros::Subscriber<std_msgs::Int32> driveSpeedSub("/drivetrain/drive", &drivetrainSpeedCallback);
// ros::Subscriber<std_msgs::Bool> driveEnableSub("/drivetrain/enable", &drivetrainEnableCallback);
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
  nh.advertise(left_wheelpod_angle_setpoint_pub);
  nh.advertise(drivetrainIsEnabledPub);
  nh.advertise(ianOutputPub);
  nh.advertise(motorSpeedPub);
  nh.subscribe(driveSpeedSub);
  // nh.subscribe(driveEnableSub);
  nh.subscribe(driveAngleSub);
}
void loop()
{
  nh.spinOnce();

  // int motor4speed = drivetrain.getSpeed(0);
  // motorSpeed.data = motor4speed;
  // motorSpeedPub.publish(&motorSpeed);

  // float left_wheelpod_angle = drivetrain.getSpeed(0);
  // left_wheelpod_angle_msg.data = left_wheelpod_angle;
  // left_wheelpod_angle_pub.publish(&left_wheelpod_angle_msg);

  // bool drivetrainIsEnabled = drivetrain.isEnabled();
  // enabbledMsg.data = drivetrainIsEnabled;
  // drivetrainIsEnabledPub.publish(&enabbledMsg);

  drivetrain.loop();

  String drivetrainWheel0Speed = String(drivetrain.getSpeed(0));
  String ianOutputString = "Motor 0: " + drivetrainWheel0Speed;
  ianOutputMsg.data = ianOutputString.c_str();
  ianOutputPub.publish(&ianOutputMsg);

  // drivetrain.setWheelSpeeds(0, 0, 0, 0);
}