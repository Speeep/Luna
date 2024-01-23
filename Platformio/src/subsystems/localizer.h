#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/HBridge.h"
#include "./peripheral/encoder.h"

class Localizer {
    public:

        Localizer();

        void init();

        void enable();

        void disable();

        float loop();

    private:

        HBridge turnMotor;
        Encoder encoder;
        bool enabled;

};