#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "rawEncoder.h"
#include "../robotMap.h"
#include "medianFilter.h"

class ConveyorCANController {
public:
    ConveyorCANController();

    void init();

    void setMotorCurrent();

    void setMotorCurrent(int);

    void speedHandlerPID();

    void updateMotorSpeeds();

    void setSpeed(float);

    float getSpeed(int);

    float getRealSpeed();

    void cutCurrent();

    String getSums();

    int getCurrent();

private:
    struct can_frame canMsgOut;
    RawEncoder motor0Encoder;
    MedianFilter mf0;
    MCP2515 mcp2515;
    long lastTime;
    float prevAngles[1];
    float errors[1];
    float prevErrors[1];
    float setSpeeds[1];
    float speeds[1];
    float realSpeeds[1];
    int filterPosition;
    float sums[1];
    int setCurrents[1];
    float speedSetpoints[1];
    int displacements[1];
    int motorCurrent0;
};