//
//  NavigationController.cpp
//  
//
//  Created by Dickface Anselm on 2013-03-14.
//

#include <math.h>

#include "NavigationController.h"

NavigationController::NavigationController(DualMotorController &dualMotorController,
                                           MappingEncoder &mappingEncoder)
  : mDualMotorController(dualMotorController),
    mMappingEncoder(mappingEncoder),
    mMoveToSavedVector(false),
    mSavingVector(false)
{

}

double
NavigationController::CalculateOptimalReturnAngle(int x, int y)
{
  // XXX: Our x and y coordinates are flipped because Dickface Anselm sucks at math.
  //      Normally you'd put atan2(y, x).
  double optimalReturnAngle = atan2(y, x) + M_PI;
  return optimalReturnAngle > M_PI ? optimalReturnAngle - 2*M_PI : optimalReturnAngle;
}

bool
NavigationController::isHandlingMotors()
{
  return mMoveToSavedVector;
}

bool
NavigationController::handleMotors()
{
  if (!isHandlingMotors()) {
    return false;
  }

  int x = mMappingEncoder.getX();
  int y = mMappingEncoder.getY();
  double heading = mMappingEncoder.getHeading();
  mOptimalReturnAngle = CalculateOptimalReturnAngle(x, y);

  unsigned int time = millis();
  static unsigned int lastTime;
  if (time - lastTime > 500) {
    lastTime = time;

    if (abs(x) <= POSITION_TOL && abs(y) <= POSITION_TOL) {
      mDualMotorController.setSpeed(MOTOR_POWER_NONE, MOTOR_POWER_NONE, true);
      mMoveToSavedVector = false;
      ClearSerialBuffer();
      return false;
    }
  }

  if (fabs(heading - mOptimalReturnAngle) > HEADING_TOL) {
    int sign = !((heading - mOptimalReturnAngle < M_PI) ^ (heading > mOptimalReturnAngle)) ? 1 : -1;
    if (mTurnStartTime == 0) mTurnStartTime = time;
    if (time - mTurnStartTime < TURN_START_TIME) {
      mDualMotorController.setSpeed(sign * MOTOR_POWER_TURN_START, -1 * sign * MOTOR_POWER_TURN_START, true);
    } else {
      mDualMotorController.setSpeed(sign * MOTOR_POWER_TURN_STEADY, -1 * sign * MOTOR_POWER_TURN_STEADY, true);
    }
  } else /** if (abs(x) > POSITION_TOL || abs(y) > POSITION_TOL) */ {
    mTurnStartTime = 0;
    mDualMotorController.setSpeed(MOTOR_POWER_FORWARD, MOTOR_POWER_FORWARD, true);
  }

  return true;
}

void
NavigationController::startSavingVector()
{
  //if (mMoveToSavedVector) {
  //  return;
  //}

  mMappingEncoder.resetPositionAndHeading();
  //mSavingVector = true;
}

void
NavigationController::moveToSavedVector()
{
  //if (!mSavingVector) {
  //  return;
  //}

  int x = mMappingEncoder.getX();
  int y = mMappingEncoder.getY();
  double heading = mMappingEncoder.getHeading();

  mMoveToSavedVector = true;
  //mSavingVector = false;
  
  Serial.println(x);
  Serial.println(y);
  Serial.println(heading);
}

void
NavigationController::ClearSerialBuffer()
{
  while (Serial.available()) Serial.read();
}