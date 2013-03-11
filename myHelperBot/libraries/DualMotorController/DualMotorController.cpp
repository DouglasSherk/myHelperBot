//
//  MotorController.cpp
//  
//
//  Created by James Anselm on 2013-02-28.
//

#include "DualMotorController.h"

DualMotorController::DualMotorController(MotorController &mL, MotorController &mR)
  : _mL(mL),
    _mR(mR),
    _maxSpeed(5000) {
        
}

void DualMotorController::init() {
    _mL.init();
    _mR.init();
}

// Update the encoder value
void DualMotorController::updateEncoders() {
    _mL.updateEncoder();
    _mR.updateEncoder();
}

// sL is in requested speed in ticks/second
void DualMotorController::setSpeed(int sL, int sR) {
    sR *= -1; //right motor is wired backwards.
    
    if (sL > _maxSpeed) {
        sL = _maxSpeed;
    }
    else if (sL < -_maxSpeed) {
        sL = -_maxSpeed;
    }
    if (sR > _maxSpeed) {
        sR = _maxSpeed;
    }
    else if (sR < -_maxSpeed) {
        sR = -_maxSpeed;
    }
    
    if (abs(sL + sR) > 500) { //if the robot is trying to turn in place, then use straight PWM instead of trying to use speed control
        _mL.setPWM(sL);
        _mR.setPWM(sR);
    }
    else {
        _mL.setSpeed(sL);
        _mR.setSpeed(sR);
    }
}
