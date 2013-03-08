//
//  MotorController.cpp
//  
//
//  Created by James Anselm on 2013-02-28.
//

#include "MotorController.h"

MotorController::MotorController(MC33926MotorShield &ms, Encoder &en)
  : _isForward(true),
    _en(en),
    _ms(ms) {
}

void MotorController::init() {
    //nothing here yet
}

// Update the requested MotorController speed in ticks/second
void MotorController::setSpeed(int s) {
    _useSpeedControl = true;
    
    if(s>_maxSpeed) {
        _speed = _maxSpeed;
    }
    else if(s<-_maxSpeed) {
        _speed = -_maxSpeed;
    }
    else {
        _speed = s;
    }
    //Serial.print("set speed: ");
    //Serial.println(_speed);
}

// adjust the pwm to get closer to the desired speed. timeElapsed is in milliseconds.
void MotorController::adjustPWM(int timeElapsed) {
    long currentIndex = _en.getIndex();
    
    _measuredSpeed = (currentIndex - _lastIndex)/(double(timeElapsed)/1000.00);
        
    if(_isForward ^ (_speed > 0)) {
        if(abs(_measuredSpeed) < 500) {
            _isForward = !_isForward; //robot is now going in same direction as motor power is asking.
        }
        else {
            _measuredSpeed *= -1;
            //Serial.print("inverted");
        }
    }
    
    //Serial.print("\tmeasured speed: ");
    //Serial.print(_measuredSpeed);
    
    int correction = (_speed-_measuredSpeed)/100; ///100
    
    //Serial.print("\tcorrection: ");
    //Serial.print(correction);
    
    _pwmValue += correction;
    if(_pwmValue < -255) {
        _pwmValue = -255;
    }
    else if(_pwmValue > 255) {
        _pwmValue = 255;
    }
    
    //Serial.print("\tpwm: ");
    //Serial.println(_pwmValue);
    
    _ms.setPWM(_pwmValue);
    _lastIndex = currentIndex;
}

void MotorController::periodicUpdate(int timeElapsed) {
    if(_useSpeedControl) {
        adjustPWM(timeElapsed);
    }
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

// speed is in ticks/second
void MotorController::setPWM(int speed) {

    _useSpeedControl = false;
    _ms.setPWM(speed/double(_maxSpeed)*255);
    
    //Serial.print("set pwm: ");
    //Serial.println(speed/double(_maxSpeed)*255);
    
}
