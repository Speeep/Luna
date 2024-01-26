/*
 * localizer.cpp
 * 
 *  Created on: Jan 23, 2024
 *      Author: Speeep
 */
#include "localizer.h"

Localizer::Localizer(){}

void Localizer::init() {
    turnMotor.init(LOCALIZER_L_PWM_PIN, LOCALIZER_R_PWM_PIN);
    encoder.init(LOCALIZER_ENCODER_ID, MULTIPLEXER_1_ID, LOCALIZER_ENCODER_START_ANGLE);
    enabled = false;
}

void Localizer::enable() {
    enabled = true; 
}

void Localizer::disable() {
    enabled = false;
}

void Localizer::setAngleSetpoint(float newAngleSetpoint) {
    angleSetpoint = newAngleSetpoint;
}

float Localizer::getAngle() {
    return encoder.getAngle();
}

void Localizer::loop() {

    // Always get Data
    angle = encoder.getAngle();

    // If enabled, control the motors, else cut current to the motors
    if (enabled) {
        turnMotor.setEffort(int((angle - angleSetpoint) * LOCALIZER_MOTOR_KP));

    } else {
        turnMotor.setEffort(0);
    }
}