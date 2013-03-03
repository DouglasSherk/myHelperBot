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
        Encoder();
        Encoder(unsigned char VIN, unsigned char GR, unsigned char IN);
    
        //PUBLIC METHODS
        void init();
        void updateIndex(bool forward);
        long getIndex();

        //private:
        unsigned char _VIN;
        unsigned char _GR;
        unsigned char _IN;
        long _sumIndex;
        long _lastIndex;
    
};

#endif
