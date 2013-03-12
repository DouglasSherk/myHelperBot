//
//  Encoder.cpp
//  
//
//  Created by James Anselm on 2013-02-28.
//

#include "Encoder.h"

Encoder::Encoder() {
    //map motor driver pins to Arduino pins that I have arbitrarily chosen.
    _VIN = 8;
    _GR = 9;
    _IN = 4;
    _sumIndex=0;
    _lastSumIndex=0;
    _lastIndex=0;
}

Encoder::Encoder(unsigned char VIN, unsigned char GR, unsigned char IN) {
    //map motor driver pins to specified Arduino pins.
    _VIN = VIN;
    _GR = GR;
    _IN = IN;
    _sumIndex=0;
    _lastSumIndex=0;
    _lastIndex=0;
}

void Encoder::init() {
    pinMode(_IN, INPUT);
    pinMode(_GR, OUTPUT);
    digitalWrite(_GR, LOW);
    pinMode(_VIN, OUTPUT);
    digitalWrite(_VIN, HIGH);
    _sumIndex = 0L;
    _lastSumIndex = 0L;
    _lastIndex = 0L;
}

// Update the encoder valuege
void Encoder::updateIndex(bool forward = true) {
    int currentIndex = digitalRead(_IN);
    
    if (currentIndex != _lastIndex) {
        _lastIndex = currentIndex;
        if(forward) {
            _sumIndex += 1;
        }
        else {
            _sumIndex -= 1;
        }
    }
}

// Get the total number of counts
long Encoder::getIndex() {
    return _sumIndex;
}

// Get the number of counts since the last time this function was called.
long Encoder::getDeltaIndex() {
    long d = _sumIndex - _lastSumIndex;
    _lastSumIndex = _sumIndex;
    return d;
}