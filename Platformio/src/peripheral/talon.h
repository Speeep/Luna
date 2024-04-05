#pragma once

#include "Arduino.h"
#include "Servo.h"
#include "../robotMap.h"

class Talon {
 public:

  Talon();

  void init(int);

  void setEffort(int effort); // Takes -100-100

 private:
    bool attached;
    Servo PWMController;
    int pin;
};