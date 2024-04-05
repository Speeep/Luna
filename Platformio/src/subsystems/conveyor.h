#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/talon.h"
#include "./peripheral/encoder.h"
#include "./peripheral/ConveyorCANController.h"

class Conveyor{
    public:
        Conveyor();

        void init();

        void enable();

        void disable();

        bool isEnabled();

        bool isAtTop();

        bool isAtBot();

        void loop();

        void setPlungeSpeed(float);

        float getPlungeSpeed();

        void setSpeed(float);

        float getConveyerSpeed();

        int getConveryorCurrent();

    private:
        Talon plungeMotor;

        // TODO - Make can_controller static and pull the update motor speeds into main.cpp
        ConveyorCANController can_controller;

        bool enabled;
        int prevEffort;
        int effort;
        bool atTop;
        bool atBot;

        float plungeSpeed;

};