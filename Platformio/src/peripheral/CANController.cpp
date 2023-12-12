#include "CANController.h"
#include "robotMap.h"

CANController::CANController() : mcp2515(MCP_CS) {}

void CANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    kalman(0.05, 16, 1023, 0);

    angles[4] = {0};
    lastAngles[4] = {0};
    positions[4] = {0};
    actualCurrents[4] = {0};
    deltaPos[4] = {0};
    sum_delta_pos[4] = {0};
    setPos[4] = {0};
    temps[4] = {0};
    i1 = 136.53333 / (1000000 / INTERVAL);
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
}

inline bool CANController::canMsgIncoming() {
  return mcp2515.readMessage(&canMsgIn) == MCP2515::ERROR_OK;
}

inline int CANController::combineBytes(unsigned char DataH, unsigned char DataL)
{
  return int(word(DataH, DataL));
}

void CANController::updateData(int motorID) {

    angles[motorID] = combineBytes(canMsgIn.data[0], canMsgIn.data[1]);
    rawSpeed = combineBytes(canMsgIn.data[2], canMsgIn.data[3]);

    speeds[motorID] = (int)kalman.getFilteredValue(rawSpeed);

    actualCurrents[motorID] = combineBytes(canMsgIn.data[4], canMsgIn.data[5]);
    temps[motorID] = canMsgIn.data[6];

    sumDeltaPos[motorID] += setPos[motorID] - positions[motorID];

    deltaPos[motorID] = angles[motorID] - lastAngles[motorID];
    if (abs(speeds[motorID]) < 1200) {

        if (deltaPos[motorID] < -8000)
        positions[motorID]++;
        else if (deltaPos[motorID] > 8000)
        positions[motorID]--;
        else
        positions[motorID] += deltaPos[motorID];
    }
    else if (abs(speeds[motorID]) < 6000) {
        if (speeds[motorID] < 0 && deltaPos[motorID] > 4000) {
        positions[motorID] -= 8192;

        } else if (-deltaPos[motorID] > 4000 && speeds[motorID] > 0) {
        positions[motorID] += 8192;

        }
    } else
        positions[motorID] += (speeds[motorID] * i1) * 8192;

    lastAngles[motorID] = angles[motorID];
    handler_PID_Speed();

}

void CANController::getCanData() {
    if (canMsgIncoming())
    {
        updateData(canMsgIn.can_id - 0x201);
    }
}