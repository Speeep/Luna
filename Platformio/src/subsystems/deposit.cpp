/*
 * deposit.cpp
 * 
 *  Created on: Mar 20, 2024
 *      Author: WigglyWalrus
 */
#include "deposit.h"

Deposit::Deposit(){}

void Deposit::init() {
    turnMotor.init(DEPOSIT_L_PWM_PIN, DEPOSIT_R_PWM_PIN);
    encoder.init(DEPOSIT_ENCODER_ID, MULTIPLEXER_1_ID, 0.0);
    enabled = false;
    angle = 0.0;
    open = false;
    error = 0;
    errorI = 0;
}

void Deposit::enable() {
    enabled = true; 
}

void Deposit::disable() {
    enabled = false;
}

bool Deposit::isEnabled() {
    return enabled;
}

float Deposit::getAngle() {
    return angle;
}

void Deposit::setOpen(bool open){
    this.open = open;
}

bool Deposit::isOpen(){
    return open;
}

bool Deposit::isInPosition(){
    if(open){
        if(getAngle >= DEPOSIT_OPEN_ANGLE){
            return true;
        }
        return false;
    }
    else{
        if(getAngle <= DEPOSIT_CLOSED_ANGLE){
            return true;
        }
        return false;
    }
}

void Deposit::loop() {

    // Always get Data
    angle = encoder.getAngle();

    if(isInPosition()){
        return;
    }

    // Accumulate error and constrain
    errorI += error;
    errorI = constrain(errors, -MAX_DEPOSIT_ERRORS, MAX_DEPOSIT_ERRORS);

    // If enabled, control the motors, else cut current to the motors
    if (enabled) {
        depositMotor.setEffort(int((error * Deposit_MOTOR_KP) + (errorI * DEPOSIT_MOTOR_KI)));
    }
    else{
        depositMotor.setEffort(0);
    }
}