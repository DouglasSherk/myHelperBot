using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace myHelperBot
{
  class mhbLogic
  {
    public mhbLogic()
    {

    }

    public void loop()
    {
      mhbState state;
      lock (mhbState.Lock) {
        state = mhbState.g;
      }

      Vector3D forwardVector = new Vector3D(0.0, 0.0, 1.0);
      Vector3D userVector = new Vector3D(state.userPosition.X,
                                         0.0 /** ignore elevation */,
                                         state.userPosition.Z);

      double rot = Vector3D.AngleBetween(forwardVector, userVector);

      if (rot > ROT_MAX) {
        state.leftSpeed = userVector.X > 0.0 ? SPEED_MAX : -SPEED_MAX;
        state.rightSpeed = userVector.X > 0.0 ? -SPEED_MAX : SPEED_MAX;
      }

      lock (mhbState.Lock) {
        mhbState.g = state;
      }
    }

    private int SPEED_NONE = 0;
    private int SPEED_MAX = 5000;

    private double DIST_MIN = 1.7;
    private double DIST_MAX = 2.0;

    private double ROT_MAX = 10.0;

    private double ROT_FACTOR = /** SPEED_MAX */ 1.0 / 1.0;
    private double DIST_FACTOR = /** SPEED_MAX */ 1.0 / 2.0;
  }
}
