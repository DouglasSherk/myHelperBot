//
//  MotorController.cpp
//  
//
//  Created by James Anselm on 2013-02-28.
//

#include "MotorController.h"

#include <math.h>

MotorController::MotorController(MC33926MotorShield &ms, Encoder &en)
  : _en(en),
    _ms(ms) {
}

void MotorController::init() {
    //nothing here.
}

// Update the requested MotorController speed in ticks/second
void MotorController::setSpeed(int s) {
    _useSpeedControl = true;
    
    if(s>MAX_SPEED) {
        _speed = MAX_SPEED;
    }
    else if(s<-MAX_SPEED) {
        _speed = -MAX_SPEED;
    }
    else {
        _speed = s;
    }
}

// adjust the pwm to get closer to the desired speed. timeElapsed is in milliseconds.
void MotorController::adjustPWM(int timeElapsed) {
    long currentIndex = _en.getIndex();
    
    _measuredSpeed = (currentIndex - _lastIndex)/(double(timeElapsed)/1000.00);
    
    Serial.print("_speed: ");
    Serial.print(_speed);
    Serial.print("\tmeasured: ");
    Serial.print(_measuredSpeed);
        
    int correction = (_speed-_measuredSpeed)/100; ///100
    
    Serial.print("\tcorrection: ");
    Serial.print(correction);
    
    if(correction>MAX_CORRECTION) {
        correction = MAX_CORRECTION;
    }
    else if(correction < -MAX_CORRECTION) {
        correction = -MAX_CORRECTION;
    }
    
    _pwmValue += correction;
    if(_pwmValue < -255) {
        _pwmValue = -255;
    }
    else if(_pwmValue > 255) {
        _pwmValue = 255;
    }
    
    Serial.print("\tpwm: ");
    Serial.println(_pwmValue);
    
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
    _en.updateIndex();
}

// Takes speed in the same units as setSpeed (ticks/s).
// While there's no direct conversions between these two since
// what we're really setting in this function is the acceleration,
// we can at least convert approximately so that both functions
// behave similarly over short times. Note that a speed of 0 stops
// the motors, and any speed above or below that should make it
// barely begin to move.
void MotorController::setPWM(int speed) {
    _useSpeedControl = false;
    int powerToGetSpeed = 0;
    if (speed) {
        // Calculate uninterpolated power, which is a direct linear
        // unit conversion from speed to power.
        double powerUninterpolated = MAX_POWER * fabs(speed) / double(MAX_SPEED);

        // Linearly interpolate power from the minimum required to
        // actually move the motors to the maximum.
        double powerInterpolated =
            MIN_POWER +
            powerUninterpolated * double(MAX_POWER - MIN_POWER) / MAX_POWER;

        powerToGetSpeed = ceil(powerInterpolated) * (speed > 0 ? 1 : -1);
    }

    if (powerToGetSpeed > MAX_POWER) powerToGetSpeed = MAX_POWER;
    else if (powerToGetSpeed < -MAX_POWER) powerToGetSpeed = -MAX_POWER;

    _ms.setPWM(powerToGetSpeed);
    _pwmValue = powerToGetSpeed;
    
    //Serial.print("set pwm: ");
    //Serial.println(speed/double(MAX_SPEED)*255);
}
