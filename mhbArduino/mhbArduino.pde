#define MOTOR_INTERVAL 100

#include "MC33926MotorShield.h"
MC33926MotorShield motorL(30, 22, 24, 2, 26, 28);
MC33926MotorShield motorR(31, 23, 25, 3, 27, 29);

#include "Encoder.h"
Encoder enL(9, 10, 11, 12, 13);
Encoder enR(4, 5, 6, 7, 8);

#include "MotorController.h"
MotorController mcL(motorL, enL);
MotorController mcR(motorR, enR);

#include "DualMotorController.h"
DualMotorController dmc(mcL, mcR);

unsigned long gMotorTimer;

void setup()
{
	Serial.begin(57600);

	motorL.init();
	motorR.init();
	enL.init();
	enR.init();
	mcR.init();
	mcL.init();

	unsigned int time = millis();
	gMotorTimer = time + 100;
}

void loop()
{
	char data[256];
	int i;

	while (Serial.available())
	{
		i = 1;
		data[0] = (char)Serial.read();
		
		while (data[i - 1] != '\0') {
			if (Serial.available()) {
				data[i++] = (char)Serial.read();
			}
		}

		int leftSpeed, rightSpeed;
		sscanf(data, "%d, %d", &leftSpeed, &rightSpeed);

		dmc.setSpeed(leftSpeed, rightSpeed, true);
	}

	unsigned int time = millis();

	dmc.updateEncoders();

	// Motor timed code.
	if (time - gMotorTimer > MOTOR_INTERVAL) {
		mcR.periodicUpdate(time - gMotorTimer);
		mcL.periodicUpdate(time - gMotorTimer);
		gMotorTimer = time;
	}
}
 