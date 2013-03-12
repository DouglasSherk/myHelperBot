#include <Arduino.h>
#include <GP2Y0A21YK0F.h>

const float GP2Y0A21YK0F::_testDistance[] = {6.0,   7.0,    8.0,    10.0,   15.0,   20.0,   25.0,     30.0,   40.0,   50.0,   80.0};
const float GP2Y0A21YK0F::_testVoltage[] =  {3.12,  3.05,   2.72,   2.2,   1.38,   1.23,   0.9,    0.85,    0.7,    0.6,    0.4};
/*const float GP2Y0A21YK0F::_testVoltage[] =  {3.15,  2.95,   2.75,   2.35,   1.65,   1.35,   1.1,    0.9,    0.7,    0.6,    0.4};*/

GP2Y0A21YK0F::GP2Y0A21YK0F(int analogPin)
{
    _analogPin = analogPin;
}

int GP2Y0A21YK0F::getDistanceRaw()
{
	return analogRead(_analogPin);
}
double GP2Y0A21YK0F::getDistanceVolt()
{
	return (getDistanceRaw()*0.0049);
}


double GP2Y0A21YK0F::getDistanceCentimeter(double volt)
{
	//double volt = getDistanceVolt();
    int numTests = sizeof _testDistance/sizeof(float);
    
    if(volt>_testVoltage[0] || volt<_testVoltage[numTests-1])
        return 0;
    
    for(int i=1; i<numTests; i++) {
        if(volt > _testVoltage[i]) {
            return _testDistance[i-1] + (_testDistance[i] - _testDistance[i-1])*(_testVoltage[i-1] - volt)/(_testVoltage[i-1] - _testVoltage[i]);
        }
    }
}