#ifndef GP2Y0A21YK0F_h
#define GP2Y0A21YK0F_h
#include <Arduino.h>

class GP2Y0A21YK0F
{
	public:
		GP2Y0A21YK0F(int analogPin);

		int getDistanceRaw();
		double getDistanceVolt();
		double getDistanceCentimeter(double volt);

	private:
		int _analogPin;
        static const float _testDistance[];
        static const float _testVoltage[];
};
#endif
