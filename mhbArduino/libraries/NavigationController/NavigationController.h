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

  /** Clears the serial buffer to prevent control from making data stale. */
  void ClearSerialBuffer();

  /** Tolerance on the heading before exiting the move-to-saved-vector state, in rads. */
  const static double HEADING_TOL = 0.1;
  /** Tolerance on the position before exiting the move-to-saved-vector state, in mm. */
  const static double POSITION_TOL = 200;
  /** Amount of time to stay in turn start, in ms. */
  const static unsigned int TURN_START_TIME = 350;

  /** Power to set on motors when starting a turn. */
  const static int MOTOR_POWER_TURN_START = 1250;
  /** Power to set on motors when turning steadily. */
  const static int MOTOR_POWER_TURN_STEADY = 1;
  /** Power to set on motors when moving forward. */
  const static int MOTOR_POWER_FORWARD = 3000;
  /** Power to set on motors when not moving. */
  const static int MOTOR_POWER_NONE = 0;

  DualMotorController &mDualMotorController;
  MappingEncoder &mMappingEncoder;

  unsigned int mTurnStartTime;

  bool mMoveToSavedVector;
  bool mSavingVector;

  double mOptimalReturnAngle;
};

#endif