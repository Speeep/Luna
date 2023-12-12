#include "CANController.h"
#include "robotMap.h"

CANController::CANController() : mcp2515(MCP_CS) {}

void CANController::init(int csPin, int intPin) {
    SPI.begin();
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

void CANController::setMotorSpeed(int motorSpeed) {
    motorSpeed = constrain(motorSpeed, -100, 100);
    motorSpeed = map(motorSpeed, -100, 100, -16384, 16384);

    motorSpeed = 16384;

    (char)(16384 / 255);

    canMsgOut.can_id = 0x201;
    canMsgOut.can_dlc = 1;
    canMsgOut.data[0] = (char)(motorSpeed / 255);
    canMsgOut.data[1] = (char)(motorSpeed % 255);
    mcp2515.sendMessage(&canMsgOut);
}