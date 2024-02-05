#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/HBridge.h"
#include "./peripheral/encoder.h"
#include "./peripheral/CANController.h"
#include <std_msgs/Float32MultiArray.h>

class Drivetrain {
 public:

  Drivetrain();

  void init();

  void enable();

  void disable();

  void loop();

  void setLeftWheelpodAngle(int);

  void setRightWheelpodAngle(int);

  void setWheelSpeeds(float, float, float, float);

  void setWheelSpeeds(float, float);
  
  void turnICC(float, float);

  float getSpeed(int);

  float getLeftWheelpodAngle();

  float getRightWheelpodAngle();

  void setLeftWheelpodAngleSetpoint(float);

  void setRightWheelpodAngleSetpoint(float);

  float getLeftWheelpodAngleSetpoint();

  float getRightWheelpodAngleSetpoint();

  void setAngle(bool);

  bool isEnabled();

  void setDriveSpeed(float);

  void setRotateSpeed(float);

  float getDriveSpeed();

  String getSums();

  std_msgs::Float32MultiArray stepOdom();


 private:
    HBridge left_turn_motor;
    HBridge right_turn_motor;
    Encoder left_wheelpod_encoder;
    Encoder right_wheelpod_encoder;
    CANController can_controller;
    bool enabled;
    float leftWheelpodAngleSetpoint;
    float rightWheelpodAngleSetpoint;
    float leftWheelpodAngle;
    float rightWheelpodAngle;
    int turnMotorEffort;
    bool isAngled;
    float driveSpeed;
    long previousOdomReadTime;
};