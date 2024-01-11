/*
 * encoder.cpp
 * 
 *  Created on: Dec 11, 2023
 *      Author: Speeep
 */

#include "encoder.h"
#include <Wire.h>

Encoder::Encoder(){}

void Encoder::init(int id, float start) {
    encoderNumber = id;
    startAngle = start;
}

float Encoder::getAngle()
{
  // Multiplexer things
  Wire.beginTransmission(0x70);
  Wire.write(1 << encoderNumber);
  if (Wire.endTransmission() != 0)
  {
    // Handle I2C communication error
    Serial.println("Error setting multiplexer channel");
    return radAngle;  // Return the last known valid value
  }

  // Encoder things
  // 7:0 - low
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return radAngle;
  }
  lowbyte = Wire.read();

  // 11:8 - high
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return radAngle;
  }
  highbyte = Wire.read();
  highbyte = highbyte << 8;

  int rawAngle = (highbyte | lowbyte) & 0x0fff;
  radAngle = static_cast<float>(rawAngle) * 0.001533981 - startAngle;

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

float Encoder::getRawAngle()
{
  // Multiplexer things
  Wire.beginTransmission(0x70);
  Wire.write(1 << encoderNumber);
  if (Wire.endTransmission() != 0)
  {
    // Handle I2C communication error
    Serial.println("Error setting multiplexer channel");
    return radAngle;  // Return the last known valid value
  }

  // Encoder things
  // 7:0 - low
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return radAngle;
  }
  lowbyte = Wire.read();

  // 11:8 - high
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return radAngle;
  }
  highbyte = Wire.read();
  highbyte = highbyte << 8;

  int rawAngle = (highbyte | lowbyte) & 0x0fff;

  return rawAngle;
}