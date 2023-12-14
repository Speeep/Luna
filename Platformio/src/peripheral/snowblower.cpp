/*
 * snowblower.cpp
 *
 *  Created on: Sep 28, 2021
 *      Author: Speeep
 */

#include "snowblower.h"
#include <Wire.h>

Snowblower::Snowblower() {}

void Snowblower::init(bool isLeft)
{
    leftSide = isLeft;

    if (leftSide) {
        pinMode(LEFT_R_PWM, OUTPUT);
        pinMode(LEFT_L_PWM, OUTPUT);
        R_PWM = LEFT_R_PWM;
        L_PWM = LEFT_L_PWM;
    } else {
        pinMode(RIGHT_R_PWM, OUTPUT);
        pinMode(RIGHT_L_PWM, OUTPUT);
        R_PWM = RIGHT_R_PWM;
        L_PWM = RIGHT_L_PWM;
    }
}

void Snowblower::setEffort(int effort)
{
    if (effort > 100) {
        effort = 100;
    } else if (effort < -100) {
        effort = -100;
    }

    if (effort > 0) {
        analogWrite(L_PWM, map(effort, 0, 100, 15, 100));
        analogWrite(R_PWM, 0);
    } else if (effort < 0) {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, map(abs(effort), 0, 100, 12, 100));
    } else if (effort == 0) {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, 0);
    }
}