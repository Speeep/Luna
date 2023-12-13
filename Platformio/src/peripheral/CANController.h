#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "../robotMap.h"
#include <Kalman.h>
#include <TimerOne.h>
#include <FastPID.h>

class CANController {
public:
    CANController();

    static CANController& instance();

    void init();

    void setMotorSpeed(int);

    void setMotorCurrent();

    inline bool canMsgIncoming();

    inline int combineBytes(unsigned char, unsigned char);

    void getCanData();

    void updateData(int);

    int calcPosition(long, int);

    // void positionHandlerPID();

    void speedHandlerPID();

    // static void staticPositionHandlerPID();

    static void staticGetCanData();

    static void staticSpeedHandlerPID();

    void setSpeed(int, int, int, int);

    int getSpeed(int);

private:
    struct can_frame canMsgOut;
    struct can_frame canMsgIn;
    MCP2515 mcp2515;
    Kalman kalman;
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
    FastPID speed_PID_0;
    FastPID speed_PID_1;
    FastPID speed_PID_2;
    FastPID speed_PID_3;
    FastPID pos_PID_0;
    FastPID pos_PID_1;
    FastPID pos_PID_2;
    FastPID pos_PID_3;
};