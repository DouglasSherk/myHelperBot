//
//  MappingEncoder.cpp
//  
//
//  Created by James Anselm on 2013-02-28.
//

#include "MappingEncoder.h"

MappingEncoder::MappingEncoder(int rad, int sep, int tpr) {
    _rad = rad; //radius of wheel
    _sep = sep; //length of wheelbase
    _tpr = tpr; //number of ticks per revolution
    _x = 0; //initial x-position
    _y = 0; //initial y-position
    _heading = 0.0; //initial heading
}

void MappingEncoder::updatePosition(int leftTicks, int rightTicks) {
    
    //convert ticks to mm.
    double left = -(2*M_PI*_rad/_tpr)*leftTicks; //negative so that going forward gives positive position.
    double right = -(2*M_PI*_rad/_tpr)*rightTicks;

    double dS = (left + right)/2;
    double dHeading = (right - left)/_sep;
    int dX = dS*cos(_heading+dHeading/2);
    int dY = dS*sin(_heading+dHeading/2);

    _heading += dHeading;
    while(_heading < -M_PI){
      _heading += 2*M_PI;
    }
    while(_heading > M_PI) {
      _heading -= 2*M_PI;
    }

    _x += dX;
    _y += dY;
}

int MappingEncoder::getX() {
    return _x; 
}

int MappingEncoder::getY() {
    return _y; 
}

double MappingEncoder::getHeading() {
    return _heading; 
}

void MappingEncoder::resetPositionAndHeading() {
    _heading = 0.0;
    _x = _y = 0;
}