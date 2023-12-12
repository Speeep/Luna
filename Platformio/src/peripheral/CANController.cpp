#include "CANController.h"
#include "robotMap.h"

CANController::CANController() : mcp2515(MCP_CS) {}

void CANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

void CANController::setMotorSpeed(int motorSpeed) {
    motorSpeed = constrain(motorSpeed, -100, 100);
    motorSpeed = map(motorSpeed, -100, 100, -16384, 16384);

    canMsgOut.can_id = 0x200;
    canMsgOut.can_dlc = 8;
    canMsgOut.data[0] = (char)(motorSpeed / 255);
    canMsgOut.data[1] = (char)(motorSpeed % 255);
    canMsgOut.data[2] = (char)(motorSpeed / 255);
    canMsgOut.data[3] = (char)(motorSpeed % 255);
    canMsgOut.data[4] = (char)(motorSpeed / 255);
    canMsgOut.data[5] = (char)(motorSpeed % 255);
    canMsgOut.data[6] = (char)(motorSpeed / 255);
    canMsgOut.data[7] = (char)(motorSpeed % 255);

    mcp2515.sendMessage(&canMsgOut);

    canMsgOut.can_id = 0x1FF;
    canMsgOut.can_dlc = 8;
    canMsgOut.data[0] = (char)(motorSpeed / 255);
    canMsgOut.data[1] = (char)(motorSpeed % 255);
    canMsgOut.data[2] = (char)(motorSpeed / 255);
    canMsgOut.data[3] = (char)(motorSpeed % 255);
    canMsgOut.data[4] = (char)(motorSpeed / 255);
    canMsgOut.data[5] = (char)(motorSpeed % 255);
    canMsgOut.data[6] = (char)(motorSpeed / 255);
    canMsgOut.data[7] = (char)(motorSpeed % 255);

    mcp2515.sendMessage(&canMsgOut);
}