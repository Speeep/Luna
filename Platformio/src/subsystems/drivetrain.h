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

  void setLeftWheelpodAngle(int);

  void setRightWheelpodAngle(int);

  void setWheelSpeeds(int, int, int, int);

  int getSpeed(int);

 private:
    Snowblower left_turn_motor;
    Snowblower right_turn_motor;
    Encoder left_wheelpod_encoder;
    Encoder right_wheelpod_encoder;
    CANController can_controller;
    bool enabled;
};