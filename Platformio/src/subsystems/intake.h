#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/HBridge.h"
#include "./peripheral/encoder.h"
#include "./peripheral/CANController.h"

class Intake{
    public:
        Intake();

        void init();

        void enable();

        void disable();

        bool isEnabled();

        void setDepthSetpoint(float);

        float getDepth();

        float getDepthSetpoint();

        void setSpeed(float);

        float getSpeed();

    private:
        HBridge plungeMotor;
        Encoder plungeEncoder;

        // TODO - Make can_controller static and pull the update motor speeds into main.cpp
        static CANController can_controller;
        Encoder speedEncoder;

        float depthSetpoint;
        float depth;
        float speed;

        bool isEnabled;

};