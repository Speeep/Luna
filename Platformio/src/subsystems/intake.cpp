/*
 * intake.cpp
 * 
 *  Created on: Mar 20, 2024
 *      Author: WigglyWalrus
 */

#include "intake.h"

Intake::Intake(){
    plungeMotor.init(INTAKE_L_PWM_PIN, INTAKE_R_PWM_PIN);

    depth = 0;
    depthSetpoint = 0;
    speed = 0;

    enabled = false;

}

void Intake::init(){

}

void Intake::enable() {
    enabled = true;
}

void Intake::disable() {
    enabled = false;
}

bool Intake::isEnabled() {
    return enabled;
}

void Intake::setDepthSetpoint(float depth){
    depthSetpoint = depth;
}

float Intake::getDepth(){
    return depth;
}

float Intake::getDepthSetpoint(){
    return depthSetpoint;
}

void Intake::setSpeed(float newSpeed){
    speed = newSpeed;
}

float Intake::getSpeed(){
    return speed;
}