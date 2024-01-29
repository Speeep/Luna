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

void Drivetrain::stepOdom(float (&output)[3]){
    //measure step time length
    long currentTime = millis();
    long deltaTime = currentTime - previousOdomReadTime;

    //Calculate distances travelled by each wheel in the previous timestep
    float wheelDisplacement[4] = {
        getSpeed(0) * deltaTime * CM_PER_TICK, 
        getSpeed(1) * deltaTime * CM_PER_TICK, 
        getSpeed(2) * deltaTime * CM_PER_TICK, 
        getSpeed(3) * deltaTime * CM_PER_TICK};
    //note: conversion from steps to cm - 4096 ticks per rotation, 63.8372 per rotation, therefore  0.015585cm/tick

    //Pre calculate trig of wheel angles
    float cosThetaL = cos(getLeftWheelpodAngle());
    float sinThetaL = sin(getLeftWheelpodAngle());
    float cosThetaR = cos(getRightWheelpodAngle());
    float sinThetaR = sin(getRightWheelpodAngle());

    //Calculate estimated new wheel positions using the wheel angles and the displacements
    float newPostion0[2] = {  ROBOT_LENGTH_CM / 2 + cosThetaL * wheelDisplacement[0],   ROBOT_WIDTH_CM / 2 - sinThetaL * wheelDisplacement[0]};
    float newPostion1[2] = { -ROBOT_LENGTH_CM / 2 + cosThetaL * wheelDisplacement[1],   ROBOT_WIDTH_CM / 2 + sinThetaL * wheelDisplacement[1]};
    float newPostion2[2] = { -ROBOT_LENGTH_CM / 2 + cosThetaL * wheelDisplacement[2],  -ROBOT_WIDTH_CM / 2 + sinThetaL * wheelDisplacement[2]};
    float newPostion3[2] = {  ROBOT_LENGTH_CM / 2 + cosThetaL * wheelDisplacement[3],  -ROBOT_WIDTH_CM / 2 - sinThetaL * wheelDisplacement[3]};


    //Calculate the average position of the new wheel positions
    output[0] = (newPostion0[0] + newPostion1[0] + newPostion2[0] + newPostion3[0]) / 4;
    output[1] = (newPostion0[1] + newPostion1[1] + newPostion2[1] + newPostion3[1]) / 4;


    //Calculate the new angle using the new wheel positions
    //Left side
    float leftAngle  = atan2(newPostion0[1] - newPostion1[1], newPostion0[0] - newPostion1[0]);

    //right side
    float rightAngle = atan2(newPostion3[1] - newPostion2[1], newPostion3[0] - newPostion2[0]);

    //add average angle to output
    output[2] = (leftAngle + rightAngle) / 2;

    previousOdomReadTime = currentTime;
}

void Drivetrain::setWheelSpeeds(float sp0, float sp1, float sp2, float sp3) {
    can_controller.setSpeed(-sp0, -sp1, sp2, sp3);
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