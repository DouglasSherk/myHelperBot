using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows.Media.Media3D;

namespace myHelperBot
{
  class mhbLogic
  {
    public mhbLogic()
    {
      ROT_FACTOR *= SPEED_MAX;
      DIST_FACTOR *= SPEED_MAX;
    }

    public void loop()
    {
      mhbCore.DebugThread("logic thread started");

      while (true) {
        mhbCore.DebugThread("logic spin");

        mhbState state;
        lock (mhbState.Lock) {
          state = mhbState.g;

          state.motors.leftSpeed = SPEED_NONE;
          state.motors.rightSpeed = SPEED_NONE;

          if (state.isInStopGesture && !state.stopped) {
            state.playStopSound = true;
            state.stopped = true;
          }

          if (state.isInGoGesture && state.stopped) {
            state.playGoSound = true;
            state.stopped = false;
          }

          if (state.isInSaveGesture) {
            state.playSaveSound = true;
            state.startSavingVector = true;
          }

          if (state.isInRelocateGesture) {
            state.playRelocateSound = true;
            state.moveToSavedVector = true;
            state.stopped = true;
          }

          if (state.isTracking && !state.stopped) {
            Vector3D forwardVector = new Vector3D(0.0, 0.0, 10.0);
            // XXX: Try z^2
            Vector3D userVector = new Vector3D(state.userPosition.X,
                                               0.0 /** ignore elevation */,
                                               state.userPosition.Z);

            //double rot = Vector3D.AngleBetween(forwardVector, userVector);
            double rot = Math.Abs(50.0 * state.userPosition.X / state.userPosition.Z);
            double dist =
              Math.Sqrt(Math.Pow(state.userPosition.X, 2.0) + Math.Pow(state.userPosition.Z, 2.0));

            bool rotGettingCloser = rot < mPreviousRot;
            bool distGettingCloser = dist > DIST_MAX ? dist < mPreviousDist : dist > mPreviousDist;

            if (rot > ROT_MAX && dist < DIST_MAX_TURN) {
              if (dist > DIST_MAX) {
                rot = ROT_FORWARD;
              }

              state.motors.leftSpeed = state.motors.rightSpeed =
                Convert.ToInt32((rot - ROT_MAX) * ROT_FACTOR);
              state.motors.leftSpeed *= userVector.X > 0.0 ? -1 : 1;
              state.motors.rightSpeed *= userVector.X > 0.0 ? 1 : -1;

              if (rotGettingCloser) {
                if (Math.Abs(state.motors.leftSpeed) > SPEED_REDUCE) {
                  state.motors.leftSpeed = Math.Sign(state.motors.leftSpeed) * (Math.Abs(state.motors.leftSpeed) - SPEED_REDUCE);
                }
                if (Math.Abs(state.motors.rightSpeed) > SPEED_REDUCE) {
                  state.motors.rightSpeed = Math.Sign(state.motors.rightSpeed) * (Math.Abs(state.motors.rightSpeed) - SPEED_REDUCE);
                }
              }

              mhbCore.DebugTracking(state.userPosition, state.motors.leftSpeed, state.motors.rightSpeed,
                                    "rotation over max (" + (userVector.X > 0.0 ? "ccw" : "cw") + ")");
            } else if (dist > DIST_MAX) {
              state.motors.leftSpeed = state.motors.rightSpeed =
                Convert.ToInt32((dist - DIST_MAX) * DIST_FACTOR);
              mhbCore.DebugTracking(state.userPosition, state.motors.leftSpeed, state.motors.rightSpeed,
                                    "distance over max");
            } else if (dist < DIST_MIN) {
              state.motors.leftSpeed = state.motors.rightSpeed =
                -1 * Convert.ToInt32((DIST_MIN - dist) * DIST_FACTOR);
              mhbCore.DebugTracking(state.userPosition, state.motors.leftSpeed, state.motors.rightSpeed,
                                    "distance under min");
            }

            if (state.motors.leftSpeed < -SPEED_MAX) {
              state.motors.leftSpeed = -SPEED_MAX;
            } else if (state.motors.leftSpeed > SPEED_MAX) {
              state.motors.leftSpeed = SPEED_MAX;
            }

            if (state.motors.rightSpeed < -SPEED_MAX) {
              state.motors.rightSpeed = -SPEED_MAX;
            } else if (state.motors.rightSpeed > SPEED_MAX) {
              state.motors.rightSpeed = SPEED_MAX;
            }

            mPreviousRot = rot;
            mPreviousDist = dist;
          }

          state.isInGoGesture = false;
          state.isInRelocateGesture = false;
          state.isInSaveGesture = false;
          state.isInStopGesture = false;

          mhbState.g = state;
        }

        Thread.Sleep(10);
      }
    }

    private const int SPEED_NONE = 0;
    /** Amount to reduce speed when getting closer. */
    private const int SPEED_REDUCE = 500;
    private const int SPEED_MAX = 5000;

    private const double DIST_MIN = 1.5;
    private const double DIST_MAX = 1.7;
    private const double DIST_MAX_TURN = 2.5;

    private const double ROT_MAX = 10.0;
    private const double ROT_FORWARD = 12.0;

    private double ROT_FACTOR = /** SPEED_MAX */ 1.0 / 30.0;
    private double DIST_FACTOR = /** SPEED_MAX */ 1.0 / 0.5;

    private double mPreviousRot;
    private double mPreviousDist;
  }
}
