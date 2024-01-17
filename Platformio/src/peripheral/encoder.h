#pragma once

#include "Arduino.h"
#include "../robotMap.h"

class Encoder {
 public:

  Encoder();

  void init(int id, float start);

  float getAngle();

  float getRawAngle();

 private:
    float startAngle;
    float radAngle;
    int lowbyte;   // raw angle 7:0
    word highbyte; // raw angle 7:0 and 11:8
    int encoderNumber;
};