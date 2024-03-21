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

    isEnabled = false;

}

void Intake::init(){

}

void Intake::enable(){
    isEnabled = true;
}

void Intake::disable(){
    isEnabled = false;
}

bool Intake::isEnabled(){
    return isEnabled;
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

void Intake::setSpeed(){
    this.speed = speed;

}

float Intake::getSpeed(){
    return speed;
}