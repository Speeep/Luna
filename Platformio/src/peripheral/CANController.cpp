#include "CANController.h"
#include "robotMap.h"

CANController::CANController() : mcp2515(MCP_CS) {}

void CANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    long lastTime = 0;

    for (int i = 0; i < 4; i++) {
        prevPositions[i] = 0;
        errors[i] = 0;
        prevErrors[i] = 0;
        setSpeeds[i] = 0;
        speeds[i] = 0;
        sums[i] = 0;
        setCurrents[i] = 0;
        speedSetpoints[i] = 0;
    }

    motor1Encoder.init();
    motor2Encoder.init();
    motor3Encoder.init();
    motor4Encoder.init();
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

void CANController::updateMotorSpeeds() {
    int motor1Angle = motor1Encoder.getAngle();
    int motor2Angle = motor2Encoder.getAngle();
    int motor3Angle = motor3Encoder.getAngle();
    int motor4Angle = motor4Encoder.getAngle();

    long time = millis();

    long deltaTime = time - lastTime;

    int motor1Speed = (motor1Angle - prevPositions[0]) / deltaTime;
    int motor2Speed = (motor2Angle - prevPositions[1]) / deltaTime;
    int motor3Speed = (motor3Angle - prevPositions[2]) / deltaTime;
    int motor4Speed = (motor4Angle - prevPositions[3]) / deltaTime;

    speeds[0] = motor1Speed;
    speeds[1] = motor2Speed;
    speeds[2] = motor3Speed;
    speeds[3] = motor4Speed;

    // Update all last timestep values
    lastTime = time;
    prevPositions[0] = motor1Angle;
    prevPositions[1] = motor2Angle;
    prevPositions[2] = motor3Angle;
    prevPositions[3] = motor4Angle;
    
}

void CANController::setSpeed(int sp0, int sp1, int sp2, int sp3) {
    setSpeeds[0] = sp0;
    setSpeeds[1] = sp1;
    setSpeeds[2] = sp2;
    setSpeeds[3] = sp3;

    speedHandlerPID();
}

void CANController::cutCurrent() {
    setCurrents[0] = 0;
    setCurrents[1] = 0;
    setCurrents[2] = 0;
    setCurrents[3] = 0;
    setMotorCurrent();
}