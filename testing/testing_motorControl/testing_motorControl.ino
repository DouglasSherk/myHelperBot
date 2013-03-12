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

int testSpeed = 2500;

void setup() {   
   Serial.begin(38400);
   motorL.init(); 
   motorR.init();
   enL.init();
   enR.init();
   mcL.init();
   mcR.init();   
   pinMode(13, OUTPUT);
   t.every(100,test);
   //t.every(1000,printSpeedData);
   t.every(2000,changeSpeed);
   //mcL.setSpeed(testSpeed);
   //mcR.setSpeed(testSpeed);
   //mc.setSpeed(testSpeed, testSpeed);
}

int count = 0;

void loop() {         
      mcL.updateEncoder();
      mcR.updateEncoder();
      t.update();
}

void changeSpeed() {
   testSpeed *= -1;
   Serial.print("ardNewSpeed: ");
   Serial.println(testSpeed);
   mc.setSpeed(testSpeed, testSpeed);  
}

void test() {
  printSpeedData();
  mcL.periodicUpdate(100);
  mcR.periodicUpdate(100);
}

void printSpeedData() {
    //Serial.print("L_req: ");
    //Serial.println(mcL._speed);
    //Serial.print("\tL_cur: ");
    //Serial.print(mcL._currentSpeed);*/
    /*Serial.print("L_ind: ");    
    Serial.print(mcL._en.getIndex());
    Serial.print("\tR_ind: ");    
    Serial.println(mcR._en.getIndex());*/
   /* Serial.print("\tL_lastInd: ");    
    Serial.print(mcL._lastIndex);
    Serial.print("\tL_pmw: ");
    Serial.println(mcL._pwmValue);*/
    /*Serial.print("\tR_req: ");
    Serial.print(mcR._speed);
    Serial.print("\tR_pmw: ");
    Serial.println(mcR._pwmValue);*/   
}
