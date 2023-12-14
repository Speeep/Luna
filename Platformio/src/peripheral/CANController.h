#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "../robotMap.h"
#include <Kalman.h>

class CANController {
public:
    CANController();

    void init();

    void setMotorSpeed(int);

    void setMotorCurrent();

    inline bool canMsgIncoming();

    inline int combineBytes(unsigned char, unsigned char);

    void getCanData();

    void updateData(int);

    int calcPosition(long, int);

    void speedHandlerPID();

    void setSpeed(int, int, int, int);

    int getSpeed(int);

    void cutCurrent();

private:
    struct can_frame canMsgOut;
    struct can_frame canMsgIn;
    MCP2515 mcp2515;
    Kalman m0kalman;
    Kalman m1kalman;
    Kalman m2kalman;
    Kalman m3kalman;
    int rawSpeed;
    int angles[4];
    int lastAngles[4];
    int speeds[4];
    long positions[4];
    int actualCurrents[4];
    int setCurrents[4];
    int deltaPos[4];
    long sumDeltaPos[4];
    long setPos[4];
    int setSpeeds[4];
    int temps[4];
    int errors[4];
    int prevErrors[4];
    int sums[4];
    float i1;
};