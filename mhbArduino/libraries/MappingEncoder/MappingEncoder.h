//
//  MappingEncoder.h
//  
//
//  Created by James Anselm on 2013-03-02.
//


#ifndef _MappingEncoder_h
#define _MappingEncoder_h

#include <Arduino.h>

#include <math.h>

class MappingEncoder
{
    public:
        //CONSTRUCTORS
        MappingEncoder(int rad, int sep, int tpr);
    
        //PUBLIC METHODS
        void updatePosition(int leftTicks, int rightTicks);
        int getX();
        int getY();
        double getHeading();
        void resetPositionAndHeading();

    private:
        int _x;
        int _y;
        double _heading;
        int _rad; //radius of wheel
        int _sep; //length of wheelbase
        int _tpr;  //number of ticks per revolution
};

#endif
