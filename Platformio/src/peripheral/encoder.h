/*
 * encoder.h
 * 
 *  Created on: Dec 11, 2023
 *      Author: Speeep
 */
#pragma once

#include "Arduino.h"
#include "../robotMap.h"

class Encoder {
 public:

  Encoder();

  void init();

  float getAngle();

 private:
  const int pin = LIMIT_SWITCH_PIN;
};