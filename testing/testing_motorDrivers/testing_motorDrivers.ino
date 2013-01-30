#include "MC33926MotorShield.h"

MC33926MotorShield m1(21, 23, 25, 3, 27, 29);
MC33926MotorShield m2(20, 22, 24, 2, 26, 28);

void setup() {
   m1.init();
   m2.init();
   Serial.begin(38400);
}

void loop() {
   /*m1.setSpeed(200);
   Serial.print("D1: ");
   Serial.print(digitalRead(m1._D1));
   Serial.print("  EN: ");
   Serial.print(digitalRead(m1._EN));
   Serial.print("  IN1: ");
   Serial.print(digitalRead(m1._IN1));
   Serial.print("  IN2: ");
   Serial.print(digitalRead(m1._IN2));
   Serial.print("  INV: ");
   Serial.print(digitalRead(m1._INV));
   Serial.print("\n");*/
   
   for(int value=0; value<255; value += 10){
      m1.setSpeed(value);
      m2.setSpeed(value);
      Serial.println(value);
      delay(200);  
   }
}
