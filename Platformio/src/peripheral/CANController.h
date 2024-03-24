#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "rawEncoder.h"
#include "../robotMap.h"
#include "medianFilter.h"

class CANController {
public:
    CANController();

    void init();

    void setMotorCurrent();

    void speedHandlerPID();

    void updateMotorSpeeds();

    void setSpeed(int, float);

    float getSpeed(int);

    float getRealSpeed(int);

    int getDisplacement(int);

    void cutCurrent();

    String getSums();

private:
    struct can_frame canMsgOut;
    RawEncoder motor0Encoder;
    RawEncoder motor1Encoder;
    RawEncoder motor2Encoder;
    RawEncoder motor3Encoder;
    RawEncoder motor4Encoder;
    MedianFilter mf0;
    MedianFilter mf1;
    MedianFilter mf2;
    MedianFilter mf3;
    MedianFilter mf4;
    MCP2515 mcp2515;
    long lastTime;
    float prevAngles[5];
    float errors[5];
    float prevErrors[5];
    float setSpeeds[5];
    float speeds[5];
    float realSpeeds[5];
    int filterPosition;
    float sums[5];
    int setCurrents[5];
    float speedSetpoints[5];
    int displacements[5];
};