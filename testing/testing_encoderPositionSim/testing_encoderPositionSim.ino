#include <math.h>
#include <MappingEncoder.h>

MappingEncoder me(178/2, 340, 2500);

void setup() {
   Serial.begin(38400);
   Serial.println("initial position: ");
   Serial.print("(");
   Serial.print(me.getX());
   Serial.print(", ");
   Serial.print(me.getY());
   Serial.print(", ");
   Serial.print(me.getHeading());
   Serial.println(")");
}

void loop() {
     me.updatePosition(0,2390);
     Serial.print("(");
     Serial.print(me.getX());
     Serial.print(", ");
     Serial.print(me.getY());
     Serial.print(", ");
     Serial.print(me.getHeading());
     Serial.println(")");
     delay(500);
}
