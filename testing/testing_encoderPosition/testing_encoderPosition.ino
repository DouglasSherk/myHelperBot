#include "MC33926MotorShield.h"
MC33926MotorShield motorL(30, 22, 24, 2, 26, 28); //left
MC33926MotorShield motorR(31, 23, 25, 3, 27, 29); //right

#include "Timer.h"
Timer t;

#include "Encoder.h"
Encoder enL(10,11,5);
Encoder enR(8,9,4);

#include "MotorController.h"
MotorController mcL(motorL, enL);
MotorController mcR(motorR, enR);

#include "DualMotorController.h"
DualMotorController mc(mcL, mcR);

#include <math.h>
#include <MappingEncoder.h>
MappingEncoder me(178/2, 340, 2500);

int testSpeed = 2500;

void setup() {   
   Serial.begin(38400);
   motorL.init(); 
   motorR.init();
   enL.init();
   enR.init();
   mcL.init();
   mcR.init();   
   t.every(100,test);
}

void loop() {         
    mc.updateEncoders();
    //enL.updateIndex(true);
    //enR.updateIndex(true);
    t.update();
}

void test() {
  updatePosition(); 
  mcL.periodicUpdate(100);
  mcR.periodicUpdate(100);
}

void updatePosition() {
     me.updatePosition(enL.getDeltaIndex(),enR.getDeltaIndex());
     Serial.print("(");
     Serial.print(me.getX());
     Serial.print(", ");
     Serial.print(me.getY());
     Serial.print(", ");
     Serial.print(me.getHeading());
     Serial.print(")\t\t");
     Serial.print("L_ind: ");    
     Serial.print(enL.getIndex());
     Serial.print("\tR_ind: ");    
     Serial.println(enR.getIndex());
}
