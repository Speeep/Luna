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
}

void Drivetrain::enable() {
    enabled = true;    
}

void Drivetrain::disable() {
    // Set wheel speeds to 0 before disabling
    can_controller.cutCurrent();
    enabled = false;
}

void Drivetrain::setWheelSpeeds(int sp0, int sp1, int sp2, int sp3) {
    if (enabled) {
        can_controller.getCanData();
        can_controller.setSpeed(-sp0, -sp1, sp2, sp3);
    } else {
        can_controller.getCanData();
        can_controller.cutCurrent();
    }
}

int Drivetrain::getSpeed(int motorId) {
    return can_controller.getSpeed(motorId);
}