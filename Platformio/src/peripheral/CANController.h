#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "../robotMap.h"
#include <Kalman.h>
#include <TimerOne.h>
#include <MsTimer2.h>
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

    void positionHandlerPID();

    void speedHandlerPID();

    static void staticPositionHandlerPID();

    static void staticGetCanData();

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
    float i1;
    float pos_Kp;
    int pos_Ki;         
    int pos_Kd;
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    FastPID speed_PID_0;
    FastPID speed_PID_1;
    FastPID speed_PID_2;
    FastPID speed_PID_3;
    FastPID pos_PID_0;
    FastPID pos_PID_1;
    FastPID pos_PID_2;
    FastPID pos_PID_3;
};