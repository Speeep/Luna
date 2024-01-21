/*
 * encoder.cpp
 * 
 *  Created on: Dec 11, 2023
 *      Author: Speeep
 */

#include "rawEncoder.h"
#include <Wire.h>

RawEncoder::RawEncoder(){}

void RawEncoder::init(int id) {
    encoderNumber = id;
    lastRawAngle = 0;
}

int RawEncoder::getRawAngle()
{
  // Multiplexer things
  Wire.beginTransmission(0x70);
  Wire.write(1 << encoderNumber);
  if (Wire.endTransmission() != 0)
  {
    // Handle I2C communication error
    Serial.println("Error setting multiplexer channel");
    return lastRawAngle;  // Return the last known valid value
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