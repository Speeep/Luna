#pragma once

#include "Arduino.h"
#include "../robotMap.h"

class Snowblower {
 public:

  Snowblower();

  void init(bool);

  void setEffort(int effort);

 private:
    bool leftSide;
    int L_PWM;
    int R_PWM;
};