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

    void setSpeed(int, int, int, int);

    void cutCurrent();

private:
    struct can_frame canMsgOut;
    Encoder motor1Encoder;
    Encoder motor2Encoder;
    Encoder motor3Encoder;
    Encoder motor4Encoder;
    MCP2515 mcp2515;
    long lastTime;
    int prevPositions[4];
    int errors[4];
    int prevErrors[4];
    int setSpeeds[4];
    int speeds[4];
    int sums[4];
    int setCurrents[4];
    int speedSetpoints[4];
};