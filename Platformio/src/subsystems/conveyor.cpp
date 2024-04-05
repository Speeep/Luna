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

    plungeMotor.init(PLUNGE_TALON_PWM);

    pinMode(PLUNGE_BOT, INPUT_PULLUP);
    pinMode(PLUNGE_TOP, INPUT_PULLUP);

    plungeSpeed = 0.0;
    prevEffort = 0;
    effort = 0;

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

bool Conveyor::isAtTop(){
    return digitalRead(PLUNGE_TOP) == LOW;
}

bool Conveyor::isAtBot(){
    return digitalRead(PLUNGE_BOT) == LOW;
}

void Conveyor::loop() {

    // Always Get Data
    can_controller.updateMotorSpeeds();

    if (enabled) {
        setSpeed(ZEB_SPEED);
    } else {
        setSpeed(0.0);
    }

    // Positive speed means plunging downwards
    // signal goes low when a limit is hit
    if(plungeSpeed > 0 && digitalRead(PLUNGE_BOT) == HIGH){
        // Run motor to plunge down
        effort = -PLUNGE_MOTOR_EFFORT;
    }
    else if(plungeSpeed < 0 && digitalRead(PLUNGE_TOP) == HIGH){
        // Run motor to plunge down
        effort = PLUNGE_MOTOR_EFFORT;
    }
    else{
        effort = 0;
    }

    if (effort != prevEffort) {
        prevEffort = effort;
        plungeMotor.setEffort(effort);
    }

}

void Conveyor::setPlungeSpeed(float speed){
    plungeSpeed = speed;
}

float Conveyor::getPlungeSpeed(){
    return plungeSpeed;
}

void Conveyor::setSpeed(float newSpeed){
    can_controller.setSpeed(newSpeed);
}

float Conveyor::getConveyerSpeed(){
    return can_controller.getRealSpeed();
}

int Conveyor::getConveryorCurrent(){
    return can_controller.getCurrent();
}