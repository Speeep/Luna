#include "CANController.h"
#include "robotMap.h"

CANController::CANController() : mcp2515(MCP_CS) {}

void CANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    angles[4] = {0};
    lastAngles[4] = {0};
    speeds[4] = {0};
    actualCurrents[4] = {0};
    temps[4] = {0};
    positions[4] = {0};
    setCurrents[4] = {0};
    deltaPos[4] = {0};
    sumDeltaPos[4] = {0};
    setPos[4] = {0};
    setSpeeds[4] = {0};
    errors[4] = {0};
    prevErrors[4] = {0};
    sums[4] = {0};
    i1 = 136.53333 / (1000000 / INTERVAL);
}

void CANController::setMotorCurrent() {
    canMsgOut.can_id = 0x200;
    canMsgOut.can_dlc = 8;

    int motorCurrent1 = constrain(setCurrents[0], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);
    int motorCurrent2 = constrain(setCurrents[1], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);
    int motorCurrent3 = constrain(setCurrents[2], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);
    int motorCurrent4 = constrain(setCurrents[3], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);

    canMsgOut.data[0] = (char)(motorCurrent1 / 256);
    canMsgOut.data[1] = (char)(motorCurrent1 % 256);
    canMsgOut.data[2] = (char)(motorCurrent2 / 256);
    canMsgOut.data[3] = (char)(motorCurrent2 % 256);
    canMsgOut.data[4] = (char)(motorCurrent3 / 256);
    canMsgOut.data[5] = (char)(motorCurrent3 % 256);
    canMsgOut.data[6] = (char)(motorCurrent4 / 256);
    canMsgOut.data[7] = (char)(motorCurrent4 % 256);

    mcp2515.sendMessage(&canMsgOut);
}

inline bool CANController::canMsgIncoming() {
  return mcp2515.readMessage(&canMsgIn) == MCP2515::ERROR_OK;
}

inline int CANController::combineBytes(unsigned char DataH, unsigned char DataL)
{
  return int(word(DataH, DataL));
}

int CANController::calcPosition(long setPos, int motorID)
{
  return constrain(
      POS_KP * (setPos - positions[motorID]) + 0 * sumDeltaPos[motorID],
      -16384, 16384);
}

void CANController::speedHandlerPID() {

    // Define Errors for this loop
    errors[0] = setSpeeds[0] - speeds[0];
    errors[1] = setSpeeds[1] - speeds[1];
    errors[2] = setSpeeds[2] - speeds[2];
    errors[3] = setSpeeds[3] - speeds[3];

    // Integrate the error over time
    sums[0] = constrain(sums[0] + errors[0], -SPEED_SUMCAP, SPEED_SUMCAP);
    sums[1] = constrain(sums[1] + errors[1], -SPEED_SUMCAP, SPEED_SUMCAP);
    sums[2] = constrain(sums[2] + errors[2], -SPEED_SUMCAP, SPEED_SUMCAP);
    sums[3] = constrain(sums[3] + errors[3], -SPEED_SUMCAP, SPEED_SUMCAP);

    // PID Here
    setCurrents[0] = SPEED_KP * errors[0] + SPEED_KI * sums[0] + SPEED_KD * (errors[0] - prevErrors[0]);
    setCurrents[1] = SPEED_KP * errors[1] + SPEED_KI * sums[1] + SPEED_KD * (errors[1] - prevErrors[1]);
    setCurrents[2] = SPEED_KP * errors[2] + SPEED_KI * sums[2] + SPEED_KD * (errors[2] - prevErrors[2]);
    setCurrents[3] = SPEED_KP * errors[3] + SPEED_KI * sums[3] + SPEED_KD * (errors[3] - prevErrors[3]);

    prevErrors[0] = errors[0];
    prevErrors[1] = errors[1];
    prevErrors[2] = errors[2];
    prevErrors[3] = errors[3];

    setMotorCurrent();
}

void CANController::updateData(int motorID) {

    angles[motorID] = combineBytes(canMsgIn.data[0], canMsgIn.data[1]);
    rawSpeed = combineBytes(canMsgIn.data[2], canMsgIn.data[3]);

    speeds[motorID] = rawSpeed;

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
}

void CANController::getCanData() {
    if (canMsgIncoming())
    {
        int motorID = canMsgIn.can_id - 0x201;
        if (motorID >= 0 && motorID < 4) {
            updateData(motorID);
        }
    }
}

void CANController::setSpeed(int sp0, int sp1, int sp2, int sp3) {
    setSpeeds[0] = sp0;
    setSpeeds[1] = sp1;
    setSpeeds[2] = sp2;
    setSpeeds[3] = sp3;

    speedHandlerPID();
}

int CANController::getSpeed(int motorID) {
    return speeds[motorID];
}

void CANController::cutCurrent() {
    setCurrents[4] = {0};
    setMotorCurrent();
}