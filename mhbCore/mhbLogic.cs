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
        }

        state.leftSpeed = SPEED_NONE;
        state.rightSpeed = SPEED_NONE;

        if (state.isInStopGesture && !state.stopped) {
          state.playStopSound = true;
          state.stopped = true;
        }

        if (state.isInGoGesture && state.stopped) {
          state.playGoSound = true;
          state.stopped = false;
        }

        if (state.isTracking && !state.stopped) {
          Vector3D forwardVector = new Vector3D(0.0, 0.0, 10.0);
          Vector3D userVector = new Vector3D(state.userPosition.X,
                                             0.0 /** ignore elevation */,
                                             state.userPosition.Z);

          //double rot = Vector3D.AngleBetween(forwardVector, userVector);
          double rot = Math.Abs(50.0 * state.userPosition.X / state.userPosition.Z);
          double dist =
            Math.Sqrt(Math.Pow(state.userPosition.X, 2.0) + Math.Pow(state.userPosition.Z, 2.0));

          bool rotGettingCloser = rot < mPreviousRot;
          bool distGettingCloser = dist > DIST_MAX ? dist < mPreviousDist : dist > mPreviousDist;

          if (rot > ROT_MAX) {
            if (dist > DIST_MAX) {
              rot = ROT_FORWARD;
            }

            state.leftSpeed = state.rightSpeed =
              Convert.ToInt32((rot - ROT_MAX) * ROT_FACTOR);
            state.leftSpeed *= userVector.X > 0.0 ? -1 : 1;
            state.rightSpeed *= userVector.X > 0.0 ? 1 : -1;

            if (rotGettingCloser) {
              if (Math.Abs(state.leftSpeed) > SPEED_REDUCE) {
                state.leftSpeed = Math.Sign(state.leftSpeed) * (Math.Abs(state.leftSpeed) - SPEED_REDUCE);
              }
              if (Math.Abs(state.rightSpeed) > SPEED_REDUCE) {
                state.rightSpeed = Math.Sign(state.rightSpeed) * (Math.Abs(state.rightSpeed) - SPEED_REDUCE);
              }
            }

            mhbCore.DebugTracking(state.userPosition, state.leftSpeed, state.rightSpeed,
                                  "rotation over max (" + (userVector.X > 0.0 ? "ccw" : "cw") + ")");
          } else if (dist > DIST_MAX) {
            state.leftSpeed = state.rightSpeed =
              Convert.ToInt32((dist - DIST_MAX) * DIST_FACTOR);
            mhbCore.DebugTracking(state.userPosition, state.leftSpeed, state.rightSpeed,
                                  "distance over max");
          } else if (dist < DIST_MIN) {
            state.leftSpeed = state.rightSpeed =
              -1 * Convert.ToInt32((DIST_MIN - dist) * DIST_FACTOR);
            mhbCore.DebugTracking(state.userPosition, state.leftSpeed, state.rightSpeed,
                                  "distance under min");
          }

          if (state.leftSpeed < -SPEED_MAX) {
            state.leftSpeed = -SPEED_MAX;
          } else if (state.leftSpeed > SPEED_MAX) {
            state.leftSpeed = SPEED_MAX;
          }

          if (state.rightSpeed < -SPEED_MAX) {
            state.rightSpeed = -SPEED_MAX;
          } else if (state.rightSpeed > SPEED_MAX) {
            state.rightSpeed = SPEED_MAX;
          }

          mPreviousRot = rot;
          mPreviousDist = dist;
        }

        state.isInGoGesture = false;
        state.isInRelocateGesture = false;
        state.isInSaveGesture = false;
        state.isInStopGesture = false;

        lock (mhbState.Lock) {
          mhbState.g = state;
        }

        Thread.Sleep(10);
      }
    }

    private const int SPEED_NONE = 0;
    /** Amount to reduce speed when getting closer. */
    private const int SPEED_REDUCE = 1000;
    private const int SPEED_MAX = 5000;

    private const double DIST_MIN = 1.5;
    private const double DIST_MAX = 1.7;

    private const double ROT_MAX = 10.0;
    private const double ROT_FORWARD = 15.0;

    private double ROT_FACTOR = /** SPEED_MAX */ 1.0 / 35.0;
    private double DIST_FACTOR = /** SPEED_MAX */ 1.0 / 0.3;

    private double mPreviousRot;
    private double mPreviousDist;
  }
}
