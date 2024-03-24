#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/HBridge.h"
#include "./peripheral/encoder.h"
#include "./peripheral/ConveyorCANController.h"

class Conveyor{
    public:
        Conveyor();

        void init();

        void enable();

        void disable();

        bool isEnabled();

        void loop();

        void setDepthSetpoint(float);

        float getDepth();

        float getDepthSetpoint();

        void setSpeed(float);

        float getConveyerSpeed();

    private:
        HBridge plungeMotor;

        // TODO - Make can_controller static and pull the update motor speeds into main.cpp
        ConveyorCANController can_controller;

        // float depthSetpoint;
        // float depth;
        // float speed;

        // bool enabled;

};