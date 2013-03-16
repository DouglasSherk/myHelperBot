#define MOTOR_INTERVAL 100
#define DATA_INTERVAL 1000

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

#include "MappingEncoder.h"
MappingEncoder me(178/2, 340, 1250);

#include "NavigationController.h"
NavigationController nc(dmc, me);

unsigned long gMotorTimer;
unsigned long gDataTimer;

enum SaveState
{
  SaveState_None = 0,
  SaveState_StartSavingVector = 1,
  SaveState_MoveToSavedVector = 2,
};

void setup()
{
	Serial.begin(115200);

	motorL.init();
	motorR.init();
	enL.init();
	enR.init();
	mcR.init();
	mcL.init();

	unsigned int time = millis();
	gMotorTimer = time + 100;
  gDataTimer = time + 100;
}

void loop()
{
	char data[256];
	int i;
  
	unsigned int time = millis();

  dmc.updateEncoders();

	// Motor timed code.
	if (time - gMotorTimer > MOTOR_INTERVAL) {
		mcR.periodicUpdate(time - gMotorTimer);
		mcL.periodicUpdate(time - gMotorTimer);
    me.updatePosition(mcL._en.getDeltaIndex(), -mcR._en.getDeltaIndex());
    if (nc.isHandlingMotors()) nc.handleMotors();
    gMotorTimer = time;
	}

  if (nc.isHandlingMotors()) {
    return;
  }

  if (Serial.available())
	{
    gDataTimer = time;

		i = 1;
		data[0] = (char)Serial.read();

    if (data[0] == '@') {
		  do {
			  if (Serial.available()) {
				  data[i++] = (char)Serial.read();
			  }
      } while (data[i - 1] != '*');
      data[i - 1] = '\0';

      SaveState saveState = SaveState_None;
		  int leftSpeed = 0, rightSpeed = 0;
		  sscanf(&data[1], "%d, %d, %d", &saveState, &leftSpeed, &rightSpeed);

		  dmc.setSpeed(leftSpeed, rightSpeed, true);

      if (saveState == SaveState_StartSavingVector) {
        nc.startSavingVector();
        return;
      } else if (saveState == SaveState_MoveToSavedVector) {
        nc.moveToSavedVector();
        return;
      }
    }
	}

  if (time - gDataTimer > DATA_INTERVAL) {
    dmc.setSpeed(0, 0, true);
  }
}
 