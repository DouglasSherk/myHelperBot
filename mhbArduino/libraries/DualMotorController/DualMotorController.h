//
//  MotorController.h
//  
//
//  Created by James Anselm on 2013-02-28.
//

#ifndef _DualMotorController_h
#define _DualMotorController_h

#include <Arduino.h>
#include <MotorController.h>

class DualMotorController
{
    public:
        //CONSTRUCTORS
        DualMotorController(MotorController &mL, MotorController &mR);
    
        //PUBLIC METHODS
        void init();
        void updateEncoders();
        void setSpeed(int sL, int sR, bool forceNoSpeedControl = false);

        //private:
        MotorController &_mL;
        MotorController &_mR;
        int _maxSpeed; //max speed in ticks/second
};

#endif
