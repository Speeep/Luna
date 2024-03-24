/*
 * intake.cpp
 * 
 *  Created on: Mar 20, 2024
 *      Author: WigglyWalrus
 */

#include "conveyor.h"

Conveyor::Conveyor(){}

void Conveyor::init(){
    can_controller.init();

    // plungeMotor.init(INTAKE_L_PWM_PIN, INTAKE_R_PWM_PIN);

    // depth = 0;
    // depthSetpoint = 0;
    // speed = 0;

    enabled = false;
}

void Conveyor::enable() {
    enabled = true;
}

void Conveyor::disable() {
    enabled = false;
}

bool Conveyor::isEnabled() {
    return enabled;
}

void Conveyor::loop() {

    // Always Get Data
    can_controller.updateMotorSpeeds();

    if (enabled) {
        setSpeed(ZEB_SPEED);
    } else {
        setSpeed(0.0);
    }
}

// void Conveyor::setDepthSetpoint(float depth){
//     depthSetpoint = depth;
// }

// float Conveyor::getDepth(){
//     return depth;
// }

// float Conveyor::getDepthSetpoint(){
//     return depthSetpoint;
// }

void Conveyor::setSpeed(float newSpeed){
    can_controller.setSpeed(newSpeed);
}

float Conveyor::getConveyerSpeed(){
    return can_controller.getRealSpeed();
}