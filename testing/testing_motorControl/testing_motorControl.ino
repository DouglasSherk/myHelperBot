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

int testSpeedIndex = 0;
int testSpeed[] = {0, 625, 0, -625};

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
   t.every(5000,changeSpeed);
}

int count = 0;

void loop() {         
      mc.updateEncoders();
      t.update();
}

void changeSpeed() {
   testSpeedIndex = (testSpeedIndex+1)%4;
   Serial.print("ardNewSpeed: ");
   Serial.println(testSpeed[testSpeedIndex]);
   mc.setSpeed(testSpeed[testSpeedIndex], testSpeed[testSpeedIndex]);  
}

void test() {
  //printSpeedData();
  mcL.periodicUpdate(100);
  mcR.periodicUpdate(100);
}

void printSpeedData() {
    Serial.print("\tL_speed: ");
    Serial.print(mcL._speed);
    Serial.print("\tL_speed: ");
    Serial.print(mcR._speed);   
    //Serial.print("\tL_cur: ");
    //Serial.print(mcL._currentSpeed);*/
    //Serial.print("L_ind: ");    
    //Serial.print(enL.getIndex());
    //Serial.print("\tR_ind: ");    
    //Serial.print(enR.getIndex());
    //Serial.print("\tL_lastInd: ");    
    //Serial.print(mcL._lastIndex);
    Serial.print("\tL_pmw: ");
    Serial.print(mcL._pwmValue);
    //Serial.print("\tR_req: ");
    //Serial.print(mcR._speed);
    Serial.print("\tR_pmw: ");
    Serial.print(mcR._pwmValue);
    Serial.println();   
}
