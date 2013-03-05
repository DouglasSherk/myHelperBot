//
//  MotorController.h
//  
//
//  Created by James Anselm on 2013-02-28.
//


#ifndef _MotorController_h
#define _MotorController_h

#include "Arduino.h"
#include <Encoder.h>
#include <MC33926MotorShield.h>
#include <Timer.h>

class MotorController
{
    public:
        //CONSTRUCTORS
        MotorController(int D1, int D2, int IN1, int IN2, int INV, int EN, int VIN, int GR, int IN);
    
        //PUBLIC METHODS
        void init();
        void setSpeed(int s);
        void updateEncoder();

        //private:
        Encoder _en;
        MC33926MotorShield _ms;
        void periodicUpdate(int timeElapsed);
        void adjustPWM(int timeElapsed);
        int _speed;
        int _pwmValue;
        long _lastIndex;
        int _currentSpeed;
        const static int _maxSpeed = 5000;
};

#endif
