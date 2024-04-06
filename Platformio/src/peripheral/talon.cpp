/*
 * talon.cpp
 *
 *  Created on: Apr 3, 2024
 *      Author: WigglyWalrus
 */

#include "talon.h"

Talon::Talon() {}

void Talon::init(int PWMpin)
{
    attached = false;
    pin = PWMpin;
}

void Talon::setEffort12(int effort)
{
    if (effort > 100) {
        effort = 100;
    } else if (effort < -100) {
        effort = -100;
    }
    if(abs(effort) < 1){
        if(attached){
            PWMController.detach();
            attached = false;
        }
    }
    else{
        if(!attached){
            PWMController.attach(pin);
            attached = true;
        }
        effort = (int)map(effort,-100,100,45,135);
        PWMController.write(effort);
    }
}
void Talon::setEffort24(int effort)
{
    if (effort > 100) {
        effort = 100;
    } else if (effort < -100) {
        effort = -100;
    }
    if(abs(effort) < 1){
        if(attached){
            PWMController.detach();
            attached = false;
        }
    }
    else{
        if(!attached){
            PWMController.attach(pin);
            attached = true;
        }
        effort = (int)map(effort,-100,100,0,180);
        PWMController.write(effort);
    }
}