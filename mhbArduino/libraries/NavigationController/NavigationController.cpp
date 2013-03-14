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
    mMoveToSavedVector(false)
{

}

bool
NavigationController::handleMotors()
{
  if (!mMoveToSavedVector) {
    return false;
  }

  int x = mMappingEncoder.getX();
  int y = mMappingEncoder.getY();
  double heading = mMappingEncoder.getHeading();

  if (abs(x) <= POSITION_TOL && abs(y) <= POSITION_TOL &&
      fabs(heading - mOptimalReturnAngle) <= HEADING_TOL) {
    mMoveToSavedVector = false;
    return false;
  }

  if (fabs(heading - mOptimalReturnAngle) > HEADING_TOL) {
    mDualMotorController.setSpeed(MOTOR_POWER_TURN, -MOTOR_POWER_TURN, true);
  } else /** if (abs(x) > POSITION_TOL || abs(y) > POSITION_TOL) */ {
    mDualMotorController.setSpeed(MOTOR_POWER_FORWARD, MOTOR_POWER_FORWARD, true);
  }

  return true;
}

void
NavigationController::startSavingVector()
{
  mMappingEncoder.resetPositionAndHeading();
}

void
NavigationController::moveToSavedVector()
{
  int x = mMappingEncoder.getX();
  int y = mMappingEncoder.getY();
  double heading = mMappingEncoder.getHeading();

  mMoveToSavedVector = true;
  mOptimalReturnAngle = atan2(y, x);
}