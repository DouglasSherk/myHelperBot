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

int testSpeed[] = {0, 2500, 0, -2500};
int testSpeedIndex = 0;

void setup() {
   Serial.begin(38400);
   motorL.init(); 
   motorR.init();
   enL.init();
   enR.init();
   mcL.init();
   mcR.init(); 
   t.every(100,printEncoderValues);
   t.every(2000,changeSpeed);
}

void changeSpeed() { //want to see if roobt position is about equal when it returns
   testSpeedIndex = (testSpeedIndex + 1)%4;
   Serial.print("ardNewSpeed: ");
   Serial.println(testSpeed[testSpeedIndex]);
   mc.setSpeed(testSpeed[testSpeedIndex], testSpeed[testSpeedIndex]);  
}

void printEncoderValues() {
   Serial.print("L: ");
   Serial.print(enL.getIndex());
   Serial.print("\tR: ");
   Serial.println(enR.getIndex()); 
}
  
void loop(){
  mc.updateEncoders();
  t.update();  
}
