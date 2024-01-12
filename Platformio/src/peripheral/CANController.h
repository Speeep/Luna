#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "encoder.h"
#include "../robotMap.h"

class CANController {
public:
    CANController();

    void init();

    void setMotorCurrent();

    void speedHandlerPID();

    void updateMotorSpeeds();

    void setSpeed(float, float, float, float);

    float getSpeed(int);

    void cutCurrent();

private:
    struct can_frame canMsgOut;
    Encoder motor0Encoder;
    Encoder motor1Encoder;
    Encoder motor2Encoder;
    Encoder motor3Encoder;
    MCP2515 mcp2515;
    long lastTime;
    float prevAngles[4];
    float errors[4];
    float prevErrors[4];
    float setSpeeds[4];
    float speeds[4];
    float sums[4];
    int setCurrents[4];
    float speedSetpoints[4];
};