//
//  NavigationController.h
//  
//
//  Created by Dickface Anselm on 2013-03-14.
//

#ifndef _NavigationController_h
#define _NavigationController_h

#include <Arduino.h>

#include <math.h>

#include "DualMotorController.h"
#include "MappingEncoder.h"

class NavigationController
{
public:
  NavigationController(DualMotorController &dualMotorController, MappingEncoder &mappingEncoder);

  // XXX: Update comments.
  bool isHandlingMotors();

  /**
   * Main loop should call this to see if NC is going to intercept outside motor commands. This also
   * acts as the loop function for NC so that it can adjust the current position and orientation if
   * it's in the move-to-saved-vector state.
   *
   * Note: It will only return true if in the move-to-saved-vector state. It also only has side effects
   * in this state.
   */
  bool handleMotors();

  /** Mapping functions. */
  void startSavingVector();
  void moveToSavedVector();

protected:
  double CalculateOptimalReturnAngle(int x, int y);

  /** Tolerance on the heading before exiting the move-to-saved-vector state, in rads. */
  const static double HEADING_TOL = 0.1;
  /** Tolerance on the position before exiting the move-to-saved-vector state, in mm. */
  const static double POSITION_TOL = 200;

  /** Power to set on motors when turning. */
  const static int MOTOR_POWER_TURN = 1500;
  /** Power to set on motors when moving forward. */
  const static int MOTOR_POWER_FORWARD = 5000;
  /** Power to set on motors when not moving. */
  const static int MOTOR_POWER_NONE = 0;

  DualMotorController &mDualMotorController;
  MappingEncoder &mMappingEncoder;

  bool mMoveToSavedVector;

  double mOptimalReturnAngle;
};

#endif