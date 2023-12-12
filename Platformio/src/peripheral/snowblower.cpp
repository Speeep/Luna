/*
 * snowblower.cpp
 *
 *  Created on: Sep 28, 2021
 *      Author: Speeep
 */

#include "snowblower.h"
#include <Wire.h>

Snowblower::Snowblower() {}

void Snowblower::init()
{
    pinMode(R_IS, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IS, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    digitalWrite(R_IS, HIGH);
    digitalWrite(L_IS, HIGH);
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
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