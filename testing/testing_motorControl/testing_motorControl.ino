
#include "Timer.h"
Timer t;

#include "MotorController.h"
//have to include MotorController includes here as well
#include "Encoder.h"
#include "MC33926MotorShield.h"

MotorController mcL(30, 22, 24, 2, 26, 28, 10, 11, 5);
MotorController mcR(31, 23, 25, 3, 27, 29, 8, 9, 4);

void setup() {   
   Serial.begin(38400); 
   mcL.init();
   mcR.init();   
   pinMode(13, OUTPUT);
   t.every(100,test);
   //t.every(1000,printSpeedData);
   mcL.setSpeed(2500);
   mcR.setSpeed(-2500);
}

int count = 0;

void loop() {         
      mcL.updateEncoder();
      mcR.updateEncoder();
      t.update();
}

void test() {
  printSpeedData();
  mcL.periodicUpdate(100);
  mcR.periodicUpdate(100);
}

void printSpeedData() {
    /*Serial.print("L_req: ");
    Serial.print(mcL._speed);
    Serial.print("\tL_cur: ");
    Serial.print(mcL._currentSpeed);*/
    Serial.print("L_ind: ");    
    Serial.print(mcL._en.getIndex());
    Serial.print("\tR_ind: ");    
    Serial.println(mcR._en.getIndex());
   /* Serial.print("\tL_lastInd: ");    
    Serial.print(mcL._lastIndex);
    Serial.print("\tL_pmw: ");
    Serial.println(mcL._pwmValue);*/
    /*Serial.print("\tR_req: ");
    Serial.print(mcR._speed);
    Serial.print("\tR_pmw: ");
    Serial.println(mcR._pwmValue);*/   
}
