/*
 * drivetrain.cpp
 * 
 *  Created on: Dec 13, 2023
 *      Author: Speeep
 */
#include "drivetrain.h"

Drivetrain::Drivetrain(){}

void Drivetrain::init() {
    left_turn_motor.init(true);
    right_turn_motor.init(false);
    left_wheelpod_encoder.init(LEFT_WHEELPOD_ENCODER_ID, LEFT_WHEELPOD_ENCODER_START_ANGLE);
    right_wheelpod_encoder.init(RIGHT_WHEELPOD_ENCODER_ID, RIGHT_WHEELPOD_ENCODER_START_ANGLE);
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
    // leftWheelpodAngle = left_wheelpod_encoder.getAngle();
    // rightWheelpodAngle = right_wheelpod_encoder.getAngle();

    // if (isAngled) {
    //     setLeftWheelpodAngleSetpoint(0.7853);
    //     setRightWheelpodAngleSetpoint(-0.7853);
    // } else {
    //     setLeftWheelpodAngleSetpoint(0.0);
    //     setRightWheelpodAngleSetpoint(0.0);
    // }

    // If enabled, control the motors, else cut current to the motors
    if (enabled) {
        // left_turn_motor.setEffort(int((leftWheelpodAngle - leftWheelpodAngleSetpoint) * LEFT_TURN_MOTOR_KP));
        // right_turn_motor.setEffort(int((rightWheelpodAngle - rightWheelpodAngleSetpoint) * RIGHT_TURN_MOTOR_KP));
        setWheelSpeeds(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
    } else {
        can_controller.cutCurrent();
    }
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

float Drivetrain::getDriveSpeed() {
    return driveSpeed;
}

float Drivetrain::getSum() {
    return can_controller.getSum();
}