#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/snowblower.h"
#include "./peripheral/encoder.h"
#include "./peripheral/CANController.h"

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

  float getSum();

 private:
    Snowblower left_turn_motor;
    Snowblower right_turn_motor;
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
};