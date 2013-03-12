#include <GP2Y0A21YK0F.h>
#include <DistanceGP2Y0A21YK.h>

GP2Y0A21YK0F testSensor(A4);
DistanceGP2Y0A21YK Dist;
int distance;
double distanceRaw, distanceCM;
double distanceVolt;

void setup() 
{
  Serial.begin(9600);
  Dist.begin(A4);
}

void loop()
{
  distanceRaw = testSensor.getDistanceRaw();
  distanceVolt = testSensor.getDistanceVolt();
  distanceCM = testSensor.getDistanceCentimeter(distanceVolt);
  distance = Dist.getDistanceCentimeter();
  /*Serial.print("Raw: ");
  Serial.print(distanceRaw); 
  Serial.print("      Volt: ");
  Serial.print(distanceVolt);*/ 
  Serial.print("      custom:    ");
  Serial.print(distanceCM); 
  Serial.print("      given:    ");
  Serial.println(distance);
  delay(200); //make it readable
}
