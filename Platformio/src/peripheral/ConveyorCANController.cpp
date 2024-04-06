#include "ConveyorCANController.h"
#include "robotMap.h"

ConveyorCANController::ConveyorCANController() : mcp2515(MCP_CS) {}

void ConveyorCANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    lastTime = 0;

    for (int i = 0; i < 1; i++) {
        prevAngles[i] = 0.0;
        errors[i] = 0.0;
        prevErrors[i] = 0.0;
        setSpeeds[i] = 0.0;
        speeds[i] = 0.0;
        realSpeeds[i] = 0.0;
        sums[i] = 0.0;
        setCurrents[i] = 0;
        speedSetpoints[i] = 0.0;
        displacements[i] = 0;
        motorCurrent0 = 0;
    }

    motor0Encoder.init(CONVEYOR_ENCODER_ID, CONVEYOR_MULTIPLEXER_ID);
}

void ConveyorCANController::setMotorCurrent() {
    canMsgOut.can_id = 0x1FF;
    canMsgOut.can_dlc = 2;

    if(setSpeeds[0] > 1) {
        motorCurrent0 = 3000;
    } 
    if(setSpeeds[0] < 1){
        motorCurrent0 = -1500;
    }
    else {
        motorCurrent0 = 0;
    }

    // int motorCurrent0 = constrain(setCurrents[0], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);

    canMsgOut.data[0] = (char)(motorCurrent0 / 256);
    canMsgOut.data[1] = (char)(motorCurrent0 % 256);

    mcp2515.sendMessage(&canMsgOut);
}

void ConveyorCANController::setMotorCurrent(int current){
    canMsgOut.can_id = 0x1FF;
    canMsgOut.can_dlc = 2;

    if(abs(current) > 100) {
        motorCurrent0 = current;
    }
    else {
        motorCurrent0 = 0;
    }

    // int motorCurrent0 = constrain(setCurrents[0], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);

    canMsgOut.data[0] = (char)(motorCurrent0 / 256);
    canMsgOut.data[1] = (char)(motorCurrent0 % 256);

    mcp2515.sendMessage(&canMsgOut);
}

void ConveyorCANController::speedHandlerPID() {

    // Define Errors for this loop
    errors[0] = setSpeeds[0] - speeds[0];

    // Integrate the error over time
    sums[0] = constrain(sums[0] + errors[0], -SPEED_SUMCAP, SPEED_SUMCAP);

    // PID Here
    if (setSpeeds[0] == 0) {
        setCurrents[0] = 0;
        sums[0] = 0;
    } else if (setSpeeds[0] > 0) {
        setCurrents[0] = BASE_CURRENT + SPEED_KP * errors[0] + SPEED_KI * sums[0];
    } else {
        setCurrents[0] = -BASE_CURRENT + SPEED_KP * errors[0] + SPEED_KI * sums[0];
    }

    prevErrors[0] = errors[0];

    setMotorCurrent();
}

void ConveyorCANController::updateMotorSpeeds() {
    float motor0Angle = motor0Encoder.getRawAngle();

    long time = millis();

    long deltaTime = time - lastTime;

    float motor0deltaAngle = motor0Angle - prevAngles[0];

    if (motor0deltaAngle > 2048) {
        motor0deltaAngle -= 4096;
    } else if (motor0deltaAngle < -2048) {
        motor0deltaAngle += 4096;
    }

    displacements[0] += motor0deltaAngle;

    float motor0Speed = motor0deltaAngle / deltaTime;

    realSpeeds[0] = motor0Speed;

    speeds[0] = mf0.filter(motor0Speed);

    // Update all last timestep values
    lastTime = time;
    prevAngles[0] = motor0Angle;
    
}

void ConveyorCANController::setSpeed(float sp0) {
    setSpeeds[0] = sp0;
    speedHandlerPID();
}

float ConveyorCANController::getSpeed(int motorId) {
    return speeds[motorId];
}

float ConveyorCANController::getRealSpeed() {
    return realSpeeds[0];
}

void ConveyorCANController::cutCurrent() {
    setCurrents[0] = 0;
    setMotorCurrent();
}

int ConveyorCANController::getCurrent() {
    return motorCurrent0;
}