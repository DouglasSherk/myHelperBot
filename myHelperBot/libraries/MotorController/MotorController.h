//
//  MotorController.h
//  
//
//  Created by James Anselm on 2013-02-28.
//


#ifndef _MotorController_h
#define _MotorController_h

#include <Arduino.h>
#include <Encoder.h>
#include <MC33926MotorShield.h>

class MotorController
{
    public:
        //CONSTRUCTORS
        MotorController(MC33926MotorShield &ms, Encoder &en);
    
        //PUBLIC METHODS
        void init();
        void setSpeed(int s);
        void updateEncoder();
        void setPWM(int pwmValue);

        //private:
        Encoder _en;
        MC33926MotorShield _ms;
        void periodicUpdate(int timeElapsed);
        void adjustPWM(int timeElapsed);
        int _speed;
        int _pwmValue;
        long _lastIndex;
        int _measuredSpeed;
        const static int _maxSpeed = 5000;
        bool _useSpeedControl;
        bool _isForward; //true is forwards, false is backwards
};

#endif
