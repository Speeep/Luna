#include "CANController.h"
#include "robotMap.h"

CANController::CANController() : mcp2515(MCP_CS), kalman(0.05, 16, 1023, 0) {}

CANController& CANController::instance() {
    static CANController instance;
    return instance;
}

void CANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    Timer1.initialize(INTERVAL);
    Timer1.attachInterrupt(staticGetCanData);

    MsTimer2::set(POS_INTERVAL, staticPositionHandlerPID);
    MsTimer2::start();

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
    i1 = 136.53333 / (1000000 / INTERVAL);

    // Position loop PID parameters
    pos_Kp = 0.05;
    pos_Ki = 0;         
    pos_Kd = 0;

    // Velocity loop PID parameters
    // Increase Ki based on load, fine-tune Kp
    speed_Kp = 18;
    speed_Ki = 10;
    speed_Kd = 0.4;

    const long speed_Hz = 1000000 / INTERVAL;
    FastPID speed_PID_0(speed_Kp, speed_Ki, speed_Kd, speed_Hz, 15, true);
    FastPID speed_PID_1(speed_Kp, speed_Ki, speed_Kd, speed_Hz, 15, true);
    FastPID speed_PID_2(speed_Kp, speed_Ki, speed_Kd, speed_Hz, 15, true);
    FastPID speed_PID_3(speed_Kp, speed_Ki, speed_Kd, speed_Hz, 15, true);

    const long pos_Hz = 1000 / POS_INTERVAL;
    FastPID pos_PID_0(pos_Kp, pos_Ki, pos_Kd, pos_Hz, 15, true);
    FastPID pos_PID_1(pos_Kp, pos_Ki, pos_Kd, pos_Hz, 15, true);
    FastPID pos_PID_2(pos_Kp, pos_Ki, pos_Kd, pos_Hz, 15, true);
    FastPID pos_PID_3(pos_Kp, pos_Ki, pos_Kd, pos_Hz, 15, true);
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

void CANController::setMotorCurrent() {
  canMsgOut.can_id = 0x200;
  canMsgOut.can_dlc = 8;

  canMsgOut.data[0] = (char)(setCurrents[0] / 256);
  canMsgOut.data[1] = (char)(setCurrents[0] % 256);
  canMsgOut.data[2] = (char)(setCurrents[1] / 256);
  canMsgOut.data[3] = (char)(setCurrents[1] % 256);
  canMsgOut.data[4] = (char)(setCurrents[2] / 256);
  canMsgOut.data[5] = (char)(setCurrents[2] % 256);
  canMsgOut.data[6] = (char)(setCurrents[3] / 256);
  canMsgOut.data[7] = (char)(setCurrents[3] % 256);

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
      pos_Kp * (setPos - positions[motorID]) + 0 * sumDeltaPos[motorID],
      -16384, 16384);
}

void CANController::positionHandlerPID()
{
  setSpeeds[0] = calcPosition(setPos[0], 0);
  setSpeeds[1] = calcPosition(setPos[1], 1);
  setSpeeds[2] = calcPosition(setPos[2], 2);
  setSpeeds[3] = calcPosition(setPos[3], 3);
} 

void CANController::staticPositionHandlerPID() {
    // Call the non-static member function on an instance
    instance().positionHandlerPID();
}

void CANController::speedHandlerPID() {
    setCurrents[0] = speed_PID_0.step(setSpeeds[0], speeds[0]);
    setCurrents[1] = speed_PID_1.step(setSpeeds[1], speeds[1]);
    setCurrents[2] = speed_PID_2.step(setSpeeds[2], speeds[2]);
    setCurrents[3] = speed_PID_3.step(setSpeeds[3], speeds[3]);
    setMotorCurrent();
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
    speedHandlerPID();

}

void CANController::getCanData() {
    if (canMsgIncoming())
    {
        updateData(canMsgIn.can_id - 0x201);
    }
}

void CANController::staticGetCanData() {
    instance().getCanData();
}