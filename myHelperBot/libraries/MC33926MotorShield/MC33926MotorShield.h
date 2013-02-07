//
//  MC33926MotorShield.h
//  
//
//  Created by James Anselm on 2013-01-22.
//  Based on DualMC33926MotorShield provided by Polulo (https://github.com/pololu/dual-mc33926-motor-shield)
//

// Use a jumper pin for the "EN" pin. It has to be set to high in order for the chip to run.
// Use a jumper pin for the "D1" pin. It has to be set to low.

#ifndef _MC33926MotorShield_h
#define _MC33926MotorShield_h

#include <Arduino.h>

class MC33926MotorShield
{
    public:
        //CONSTRUCTORS
        MC33926MotorShield();
        //MC33926MotorShield(unsigned char D1, unsigned char D2, unsigned char IN1, unsigned char IN2, unsigned char INV, unsigned char EN);
         MC33926MotorShield(unsigned char INV, unsigned char EN, unsigned char D1, unsigned char D2, unsigned char IN1, unsigned char IN2);
    
        //PUBLIC METHODS
        void init();
        void setSpeed(int speed);

        //private:
        unsigned char _D1;
        unsigned char _D2;
        unsigned char _IN1;
        unsigned char _IN2;
        unsigned char _INV;
        unsigned char _EN;
};

#endif
