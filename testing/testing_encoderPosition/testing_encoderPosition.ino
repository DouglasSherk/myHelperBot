#include "MC33926MotorShield.h"
MC33926MotorShield motorL(30, 22, 24, 2, 26, 28); //left
MC33926MotorShield motorR(31, 23, 25, 3, 27, 29); //right

#include "Timer.h"
Timer t;

#include "Encoder.h"
Encoder enL(9,10,11,12,13);
Encoder enR(4,5,6,7,8);

#include "MotorController.h"
MotorController mcL(motorL, enL);
MotorController mcR(motorR, enR);

#include "DualMotorController.h"
DualMotorController mc(mcL, mcR);

#include <math.h>
#include <MappingEncoder.h>
MappingEncoder me(178/2, 340, 1250);

int testSpeed[] = {0, 1500, 0};
int testSpeedIndex = -1;

void setup() {   
   Serial.begin(38400);
   motorL.init(); 
   motorR.init();
   enL.init();
   enR.init();
   mcL.init();
   mcR.init();   
   t.every(100,test);
   //t.every(1000,changeSpeed);
}

void loop() {         
    mc.updateEncoders();
    t.update();
}

void test() {
  updatePosition(); 
  mcL.periodicUpdate(100);
  mcR.periodicUpdate(100);
}

void changeSpeed() { //want to see if roobt position is about equal when it returns
   testSpeedIndex +=1;
   if(testSpeedIndex < 3){
   Serial.print("ardNewSpeed: ");
   Serial.println(testSpeed[testSpeedIndex]);
   mc.setSpeed(testSpeed[testSpeedIndex], testSpeed[testSpeedIndex]);  
   }
}

void updatePosition() {
     me.updatePosition(enL.getDeltaIndex(),-enR.getDeltaIndex());
     Serial.print("(");
     Serial.print(me.getX());
     Serial.print(", ");
     Serial.print(me.getY());
     Serial.print(", ");
     Serial.print(me.getHeading());
     Serial.print(")");
     Serial.print("\t\t");
     /*Serial.print("L_ind: ");    
     Serial.print(enL.getIndex());
     Serial.print("\tR_ind: ");    
     Serial.print(enR.getIndex());
     Serial.print("\tLspeed: ");
     Serial.print(mc._mL._speed);
     Serial.print("\tRspeed: ");
     Serial.println(mc._mR._speed);*/  
}
