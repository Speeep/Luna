#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "../robotMap.h"

class CANController {
public:
    CANController();

    void init();

    void setMotorSpeed(int);

private:
    struct can_frame canMsgOut;
    MCP2515 mcp2515;
};