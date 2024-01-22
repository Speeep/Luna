/*
 * encoder.cpp
 * 
 *  Created on: Dec 11, 2023
 *      Author: Speeep
 */

#include "rawEncoder.h"
#include <Wire.h>

RawEncoder::RawEncoder(){}

void RawEncoder::init(int id, int multiplexerId) {
    encoderNumber = id;
    multiplexerNumber = multiplexerId;
    lastRawAngle = 0;
}

int RawEncoder::getRawAngle()
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
  // 7:0 - low
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return lastRawAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return lastRawAngle;
  }
  lowbyte = Wire.read();

  // 11:8 - high
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return lastRawAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return lastRawAngle;
  }
  highbyte = Wire.read();
  highbyte = highbyte << 8;

  if (((highbyte | lowbyte) & 0x0fff) == 0) {
    return lastRawAngle;
  } else {
    int rawAngle = (highbyte | lowbyte) & 0x0fff;
    lastRawAngle = rawAngle;
    return rawAngle;
  }

}