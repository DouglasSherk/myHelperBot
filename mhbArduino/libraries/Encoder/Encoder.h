//
//  Encoder.h
//  
//
//  Created by James Anselm on 2013-02-28.
//


#ifndef _Encoder_h
#define _Encoder_h

#include <Arduino.h>

class Encoder
{
    public:
        //CONSTRUCTORS
        Encoder(unsigned char VIN, unsigned char GR, unsigned char IN, unsigned char CHA, unsigned char CHB);
    
        //PUBLIC METHODS
        void init();
        void updateIndex();
        long getIndex();
        long getDeltaIndex();

        //private:
        unsigned char _VIN;
        unsigned char _GR;
        unsigned char _IN;
        unsigned char _CHA;
        unsigned char _CHB;
        long _sumIndex;
        long _lastSumIndex;
        long _lastCHA;
};

#endif