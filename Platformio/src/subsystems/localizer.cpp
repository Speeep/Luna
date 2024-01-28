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
    angle = 0.0;
    lastAngle = 0.0;
    angleSetpoint = 0.0;
    error = 0.0;
    hysteresis = false;
    turnAround = false;
    turnClockwise = false;
    turning = true;
}

void Localizer::enable() {
    enabled = true; 
}

void Localizer::disable() {
    enabled = false;
}

bool Localizer::isEnabled() {
    return enabled;
}

void Localizer::setError(float newError) {
    error = newError;
}

float Localizer::getError() {
    return error;
}


float Localizer::getAngle() {
    return angle;
}

void Localizer::loop() {

    // Always get Data
    angle = encoder.getAngle();

    // If enabled, control the motors, else cut current to the motors
    if (enabled) {

        if ((angle > 3 && lastAngle < -3) || (angle < -3 && lastAngle > 3)) {
            hysteresis = !hysteresis;
        }

        if (!turning && hysteresis && (abs(angle) < (PI - HYSTERESIS_LIM))) {
            turnAround = true;
            if (angle < 0) {
                angleSetpoint = angle + HYSTERESIS_BUFFER;
            }
            if (angle > 0) {
                angleSetpoint = angle - HYSTERESIS_BUFFER;
            }
        }

        if (turnAround) {
            if (angle > 0) {
                turnClockwise = true;
                turnAround = false;
                turning = true;
            }
            else if (angle < 0) {
                turnClockwise = false;
                turnAround = false;
                turning = true;
            }

        } else if (turnClockwise && turning) {
            turnMotor.setEffort(-100);
            if ((angle < angleSetpoint) && (angle > angleSetpoint - HYSTERESIS_BUFFER)) {
                turning = false;
                hysteresis = false;
            }
        } else if (!turnClockwise && turning) {
            turnMotor.setEffort(100);
            if ((angle > angleSetpoint) && (angle < angleSetpoint + HYSTERESIS_BUFFER)) {
                turning = false;
                hysteresis = false;
            }

        } else {
            turnMotor.setEffort(int((error) * LOCALIZER_MOTOR_KP));
        }

    } else {
        turnMotor.setEffort(0);
    }

    // At the end of each loop, update last angle. 
    lastAngle = angle;
}