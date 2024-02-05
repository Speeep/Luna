/*
 * drivetrain.cpp
 * 
 *  Created on: Dec 13, 2023
 *      Author: Speeep
 */
#include "drivetrain.h"

Drivetrain::Drivetrain(){}

void Drivetrain::init() {
    left_turn_motor.init(LEFT_R_PWM, LEFT_L_PWM);
    right_turn_motor.init(RIGHT_R_PWM, RIGHT_L_PWM);
    left_wheelpod_encoder.init(LEFT_WHEELPOD_ENCODER_ID, MULTIPLEXER_0_ID, LEFT_WHEELPOD_ENCODER_START_ANGLE);
    right_wheelpod_encoder.init(RIGHT_WHEELPOD_ENCODER_ID, MULTIPLEXER_0_ID, RIGHT_WHEELPOD_ENCODER_START_ANGLE);
    can_controller.init();
    enabled = false;
    leftWheelpodAngleSetpoint = 0.0;
    rightWheelpodAngleSetpoint = 0.0;
    leftWheelpodAngle = 0;
    rightWheelpodAngle = 0;
    turnMotorEffort = 0;
    driveSpeed = 0.0;
}

void Drivetrain::enable() {
    enabled = true;    
}

void Drivetrain::disable() {
    // Set wheel speeds to 0 before disabling
    can_controller.cutCurrent();
    enabled = false;
}

void Drivetrain::loop() {

    // Always Get Data
    can_controller.updateMotorSpeeds();
    leftWheelpodAngle = left_wheelpod_encoder.getAngle();
    rightWheelpodAngle = right_wheelpod_encoder.getAngle();

    if (isAngled) {
        setLeftWheelpodAngleSetpoint(0.7853);
        setRightWheelpodAngleSetpoint(-0.7853);
    } else {
        setLeftWheelpodAngleSetpoint(0.0);
        setRightWheelpodAngleSetpoint(0.0);
    }

    // If enabled, control the motors, else cut current to the motors
    if (enabled) {
        left_turn_motor.setEffort(int((leftWheelpodAngle - leftWheelpodAngleSetpoint) * LEFT_TURN_MOTOR_KP));
        right_turn_motor.setEffort(int((rightWheelpodAngle - rightWheelpodAngleSetpoint) * RIGHT_TURN_MOTOR_KP));

        if (isAngled) {
            setWheelSpeeds(-driveSpeed, -driveSpeed, driveSpeed, driveSpeed);
        } else {
            setWheelSpeeds(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
        }

    } else {
        can_controller.cutCurrent();
    }
}

std_msgs::Float32MultiArray Drivetrain::stepOdom(){

    std_msgs::Float32MultiArray output;

    // Set proper length of output.data
    output.data_length = 3;

    // Measure step time length
    long currentTime = millis();
    long deltaTime = currentTime - previousOdomReadTime;

    // Calculate distances travelled by each wheel in the previous timestep
    float wheelDisplacement[4] = {
        getSpeed(0) * deltaTime * CM_PER_TICK, 
        getSpeed(1) * deltaTime * CM_PER_TICK, 
        getSpeed(2) * deltaTime * CM_PER_TICK, 
        getSpeed(3) * deltaTime * CM_PER_TICK};
    // float wheelDisplacement[4] = {
    //     0 * deltaTime * CM_PER_TICK, 
    //     0 * deltaTime * CM_PER_TICK, 
    //     0 * deltaTime * CM_PER_TICK, 
    //     0 * deltaTime * CM_PER_TICK};
    // Note: conversion from steps to cm - 4096 ticks per rotation, 63.8372 per rotation, therefore  0.015585cm/tick

    // Pre calculate trig of wheel angles
    float cosThetaL = cos(getLeftWheelpodAngle());
    float sinThetaL = sin(getLeftWheelpodAngle());
    float cosThetaR = cos(getRightWheelpodAngle());
    float sinThetaR = sin(getRightWheelpodAngle());

    // Calculate estimated new wheel positions using the wheel angles and the displacements
    float newPostion0[2] = {  ROBOT_LENGTH_CM / 2 + cosThetaL * wheelDisplacement[0],   ROBOT_WIDTH_CM / 2 - sinThetaL * wheelDisplacement[0]};
    float newPostion1[2] = { -ROBOT_LENGTH_CM / 2 + cosThetaL * wheelDisplacement[1],   ROBOT_WIDTH_CM / 2 + sinThetaL * wheelDisplacement[1]};
    float newPostion2[2] = { -ROBOT_LENGTH_CM / 2 + cosThetaR * wheelDisplacement[2],  -ROBOT_WIDTH_CM / 2 + sinThetaR * wheelDisplacement[2]};
    float newPostion3[2] = {  ROBOT_LENGTH_CM / 2 + cosThetaR * wheelDisplacement[3],  -ROBOT_WIDTH_CM / 2 - sinThetaR * wheelDisplacement[3]};


    // Calculate the average position of the new wheel positions (rounded to 4 places)
    output.data[0] = (newPostion0[0] + newPostion1[0] + newPostion2[0] + newPostion3[0]) / 4; 
    output.data[1] = (newPostion0[1] + newPostion1[1] + newPostion2[1] + newPostion3[1]) / 4;


    // Calculate the new angle using the new wheel positions
    // Left side
    float leftAngle  = atan2(newPostion0[1] - newPostion1[1], newPostion0[0] - newPostion1[0]);

    // Right side
    float rightAngle = atan2(newPostion3[1] - newPostion2[1], newPostion3[0] - newPostion2[0]);

    // Add average angle to output (rounded to 4 places)
    output.data[2] = (leftAngle + rightAngle) / 2;

    previousOdomReadTime = currentTime;

    return output;
}

void Drivetrain::setWheelSpeeds(float sp0, float sp1, float sp2, float sp3) {
    can_controller.setSpeed(-sp0, -sp1, sp2, sp3);
}

woid Drivetrain::setWheelSpeeds(float speedL, float speedR){
    //allows for setting left and right wheel speeds
    setWheelSpeeds(speedL, speedL, speedR, speedR);
}

void Drivetrain::turnICC(float yICC, float topSpeed){
    // Calculate angles
    
    // float thetaR = atan2((ROBOT_LENGTH_CM/2), (- yICC - (ROBOT_WIDTH_CM/2)));
    // float thetaL = atan2((ROBOT_LENGTH_CM/2), (- yICC + (ROBOT_WIDTH_CM/2)));
    float thetaR = -atan2((ROBOT_WIDTH_CM / 2) + yICC,   ROBOT_LENGTH_CM / 2);
    //note: right angle is inverted from our calculations, this math assumes turning the front wheels inwards is positive theta for L and R. If confused, ask Ian
    float thetaL = atan2((ROBOT_WIDTH_CM / 2) - yICC,   ROBOT_LENGTH_CM / 2);

    //limit angles to be from -pi/2 to pi/2
    if (thetaR > HALF_PI){
        thetaR -= PI;
    }

    else if(thetaR < -HALF_PI){
        thetaR += PI;
    }

    if (thetaL > HALF_PI){
        thetaL -= PI;
    }

    else if(thetaL < HALF_PI){
        thetaL += PI;
    }

    float radiusR = sqrt(pow((double)ROBOT_LENGTH_CM / 2, 2) + pow(((double)ROBOT_WIDTH_CM/2) + yICC, 2));
    
    float radiusL = sqrt(pow((double)ROBOT_LENGTH_CM / 2, 2) + pow(((double)ROBOT_WIDTH_CM/2) - yICC, 2));

    float speedL = topSpeed;
    float speedR = topSpeed;

    //Adjust speeds to match differential wheel speeds
    if(radiusL > radiusR){
        speedR *= radiusR / radiusL;
    }
    if(radiusR > radiusL){
        speedL *= radiusL / radiusR;
    }

    //correct speeds in case icc is between wheels
    if(0 <= yICC && yICC < ROBOT_WIDTH_CM / 2){
        speedL *= -1;
    }
    if(0 > yICC && yICC > ROBOT_WIDTH_CM / -2){
        speedR *= -1;
    }

    setLeftWheelpodAngleSetpoint(thetaL);
    setRightWheelpodAngleSetpoint(thetaR);
    setWheelSpeeds(speedL, speedR);
    
}

float Drivetrain::getSpeed(int motorId) {
    return can_controller.getSpeed(motorId);
}

float Drivetrain::getLeftWheelpodAngle() {
    return left_wheelpod_encoder.getAngle();
}

float Drivetrain::getRightWheelpodAngle() {
    return right_wheelpod_encoder.getAngle();
}

void Drivetrain::setLeftWheelpodAngleSetpoint(float newSetPoint) {
    leftWheelpodAngleSetpoint = newSetPoint;
}

void Drivetrain::setRightWheelpodAngleSetpoint(float newSetPoint) {
    rightWheelpodAngleSetpoint = newSetPoint;
}

float Drivetrain::getLeftWheelpodAngleSetpoint() {
    return leftWheelpodAngleSetpoint;
}

float Drivetrain::getRightWheelpodAngleSetpoint() {
    return rightWheelpodAngleSetpoint;
}

void Drivetrain::setAngle(bool angle) {
    isAngled = angle;
}

bool Drivetrain::isEnabled() {
    return enabled;
}

void Drivetrain::setDriveSpeed(float speed) {
    driveSpeed = speed;
}

void Drivetrain::setRotateSpeed(float speed) {
    driveSpeed = speed;
}

float Drivetrain::getDriveSpeed() {
    return driveSpeed;
}

String Drivetrain::getSums() {
    return can_controller.getSums();
}

