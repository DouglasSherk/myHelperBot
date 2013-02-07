//
//  MC33926MotorShield.cpp
//  
//
//  Created by James Anselm on 2013-01-22.
//  Based on DualMC33926MotorShield provided by Polulo (https://github.com/pololu/dual-mc33926-motor-shield)
//

#include "MC33926MotorShield.h"

MC33926MotorShield::MC33926MotorShield() {
    //map motor driver pins to Arduino pins that I have arbitrarily chosen.
    _D1 = 9;
    _D2 = 4;
    _IN1 = 3;
    _IN2 = 2;
    _INV = 10;
    _EN = 8;
}

/*MC33926MotorShield::MC33926MotorShield(unsigned char D1, unsigned char D2,
unsigned char IN1, unsigned char IN2, unsigned char INV,
unsigned char EN) {*/
MC33926MotorShield::MC33926MotorShield(unsigned char INV, unsigned char EN,
                                       unsigned char D1, unsigned char D2, unsigned char IN1,
                                       unsigned char IN2) {
    //map motor driver pins to specified Arduino pins.
    _D1 = D1;
    _D2 = D2;
    _IN1 = IN1;
    _IN2 = IN2;
    _INV = INV;
    _EN = EN;
}

void MC33926MotorShield::init() {
    pinMode(_D1, OUTPUT);
    pinMode(_D2, OUTPUT);
    pinMode(_IN1, OUTPUT);
    pinMode(_IN2, OUTPUT);
    pinMode(_INV, OUTPUT);
    pinMode(_EN, OUTPUT);
    
    digitalWrite(_D1, LOW);
    digitalWrite(_IN1, HIGH);
    digitalWrite(_IN2, LOW);
    digitalWrite(_EN, HIGH);
}

// Set speed for motor, speed is a number betwenn -400 and 400
void MC33926MotorShield::setSpeed(int speed) {
    unsigned char reverse = 0;
    if(speed < 0){
        speed = -speed;
        reverse = 1;
    }
    if(speed > 255)
        speed = 255;
    if(reverse)
        digitalWrite(_INV, HIGH);
    else
        digitalWrite(_INV, LOW);
    analogWrite(_D2, speed);
}