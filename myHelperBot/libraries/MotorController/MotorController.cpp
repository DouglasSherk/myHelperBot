//
//  MotorController.cpp
//  
//
//  Created by James Anselm on 2013-02-28.
//

#include "MotorController.h"

MotorController::MotorController(int D1, int D2, int IN1, int IN2, int INV, int EN, int VIN, int GR, int IN) {
    _ms = MC33926MotorShield(D1,D2,IN1,IN2,INV,EN);
    _en = Encoder(VIN, GR, IN);
}

void MotorController::init() {
    _ms.init();
    _en.init();
}

// Update the requested MotorController speed in ticks/second
void MotorController::setSpeed(int s) {
    if(s>_maxSpeed) {
        _speed = _maxSpeed;
    }
    else if(s<-_maxSpeed) {
        _speed = -_maxSpeed;
    }
    else {
        _speed = s;
    }
    _pwmValue = 255*_speed/_maxSpeed;
    _ms.setPWM(_pwmValue);
}

// adjust the pwm to get closer to the desired speed. timeElapsed is in milliseconds.
void MotorController::adjustPWM(int timeElapsed) {
    long currentIndex = _en.getIndex();
    
    /*Serial.print("curIndex: ");
    Serial.print(currentIndex);
    Serial.print("\tlastIndex: ");
    Serial.print(_lastIndex);
    Serial.print("\ttimeElapsed: ");
    Serial.print(timeElapsed);*/
    
    _currentSpeed = (currentIndex - _lastIndex)/(double(timeElapsed)/1000.00);
    
    /*Serial.print("\tcurSpeed: ");
    Serial.println(_currentSpeed);*/
    
    int correction = abs((_currentSpeed-_speed)/100); ///100
    
    if(_currentSpeed > _speed) {
        _pwmValue -= correction;
        if(_pwmValue < -255) {
            _pwmValue = -255;
        }
    }
    else {
        _pwmValue += correction;
        if(_pwmValue > 255) {
            _pwmValue = 255;
        }
    }
    _ms.setPWM(_pwmValue);
    _lastIndex = currentIndex;
}

void MotorController::periodicUpdate(int timeElapsed) {
    adjustPWM(timeElapsed);
}

// Update the encoder value
void MotorController::updateEncoder() {
    if(_speed > 0) {
        _en.updateIndex(true);
    }
    else {
        _en.updateIndex(false);
    }
}