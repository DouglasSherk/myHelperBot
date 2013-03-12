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
        const static int MAX_SPEED = 5000;

        const static int MIN_POWER = 170;
        const static int MAX_POWER = 255;
        const static int MAX_CORRECTION = 25;
    
        Encoder &_en;
        MC33926MotorShield &_ms;
        void periodicUpdate(int timeElapsed);
        void adjustPWM(int timeElapsed);
        int _speed;
        int _pwmValue;
        long _lastIndex;
        int _measuredSpeed;
        bool _useSpeedControl;
        bool _isForward; //true is forwards, false is backwards
};

#endif
