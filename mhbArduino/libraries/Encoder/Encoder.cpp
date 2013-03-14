//
//  Encoder.cpp
//  
//
//  Created by James Anselm on 2013-02-28.
//

#include "Encoder.h"

Encoder::Encoder(unsigned char VIN, unsigned char GR, unsigned char IN, unsigned char CHA, unsigned char CHB) {
    //map motor driver pins to specified Arduino pins.
    _VIN = VIN;
    _GR = GR;
    _IN = IN;
    _CHA = CHA;
    _CHB = CHB;
}

void Encoder::init() {
    pinMode(_IN, INPUT);
    pinMode(_CHA, INPUT);
    pinMode(_CHB, INPUT);
    pinMode(_GR, OUTPUT);
    digitalWrite(_GR, LOW);
    pinMode(_VIN, OUTPUT);
    digitalWrite(_VIN, HIGH);
    _sumIndex = 0L;
    _lastSumIndex = 0L;
    _lastCHA = 0L;
}

// Update the encoder value

void Encoder::updateIndex() {
    int currentCHA = digitalRead(_CHA);
 
    if((_lastCHA == LOW) && (currentCHA == HIGH)) {
        if(digitalRead(_CHB) == LOW) {
            _sumIndex++;
        }
        else {
            _sumIndex--;
        }
    }
    _lastCHA = currentCHA;
}

/*void Encoder::updateIndex(bool forward = true) {
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
}*/
/*
void loop() {
    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
        if (digitalRead(encoder0PinB) == LOW) {
            encoder0Pos--;
        } else {
            encoder0Pos++;
        }
        Serial.print (encoder0Pos);
        Serial.print ("/");
    }
    encoder0PinALast = n;
}*/




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