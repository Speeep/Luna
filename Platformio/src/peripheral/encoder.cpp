/*
 * encoder.cpp
 * 
 *  Created on: Dec 11, 2023
 *      Author: Speeep
 */

#include "encoder.h"
#include <Wire.h>

Encoder::Encoder(){}

void Encoder::init(int id, int multiplexerId, float start) {
    encoderNumber = id;
    multiplexerNumber = multiplexerId;
    startAngle = start;
    rawAngle = 0;
    radAngle = 0.0;
}

float Encoder::getAngle()
{
  // Multiplexer things
  if(multiplexerNumber == 0){
    Wire.beginTransmission(0x70);
    Wire.write(1 << encoderNumber);
    Wire.endTransmission();
    Wire.beginTransmission(0x71);
    Wire.write(1 << 7);
    Wire.endTransmission();
  }
  else{
    Wire.beginTransmission(0x71);
    Wire.write(1 << encoderNumber);
    Wire.endTransmission();
    Wire.beginTransmission(0x70);
    Wire.write(1 << 7);
    Wire.endTransmission();
  }

  // Encoder things

  // 7:0 - low byte
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);

  if (Wire.endTransmission() != 0)
  {
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);

  if (Wire.available() == 0)
  {
    return radAngle;
  }
  lowbyte = Wire.read();

  // 11:8 - high
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0)
  {
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    return radAngle;
  }
  highbyte = Wire.read();
  highbyte = highbyte << 8;

  int rawAngle = (highbyte | lowbyte) & 0x0fff;

  if ((static_cast<float>(rawAngle) * BYTES_2_RAD - startAngle) == 0.0) {
    return radAngle;
  } 
  else {
    radAngle = static_cast<float>(rawAngle) * BYTES_2_RAD - startAngle;

    if (radAngle < 0.0)
    {
      radAngle += 6.28319;
    }
    
    if (radAngle > 3.14159)
    {
      radAngle -= 6.28319;
    }

  return radAngle;
  }
}