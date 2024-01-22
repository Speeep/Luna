#pragma once

#include "Arduino.h"
#include "../robotMap.h"

class Encoder {
 public:

  Encoder();

  void init(int id, int multiplexerId, float start);

  float getAngle();

 private:
    float startAngle;
    float radAngle;
    float rawAngle;
    int lowbyte;   // raw angle 7:0
    word highbyte; // raw angle 7:0 and 11:8
    int encoderNumber;
    int multiplexerNumber;
};