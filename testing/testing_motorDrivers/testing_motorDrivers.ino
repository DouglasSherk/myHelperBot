#include "MC33926MotorShield.h"

MC33926MotorShield motorR(31, 23, 25, 3, 27, 29); //right
MC33926MotorShield motorL(30, 22, 24, 2, 26, 28); //left

void setup() {
   motorR.init();
   motorL.init();
   Serial.begin(38400);
}

void loop() {
   /*motorR.setSpeed(200);
   Serial.print("D1: ");
   Serial.print(digitalRead(motorR._D1));
   Serial.print("  EN: ");
   Serial.print(digitalRead(motorR._EN));
   Serial.print("  IN1: ");
   Serial.print(digitalRead(motorR._IN1));
   Serial.print("  IN2: ");
   Serial.print(digitalRead(motorR._IN2));
   Serial.print("  INV: ");
   Serial.print(digitalRead(motorR._INV));
   Serial.print("\n");*/
   
      motorR.setPWM(255);
      motorL.setPWM(-255);

}
