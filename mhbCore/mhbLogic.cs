using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace myHelperBot
{
  class mhbLogic
  {
    public mhbLogic()
    {

    }

    public void loop()
    {
      int leftSpeed = 0, rightSpeed = 0;
    }

    private double SPEED_NONE = 0.0;
    private double SPEED_MAX = 5000.0;

    private double DIST_MIN = 1.7;
    private double DIST_MAX = 2.0;

    private double ROT_MAX = 0.2;

    private double ROT_FACTOR = /** SPEED_MAX */ 1.0 / 1.0;
    private double DIST_FACTOR = /** SPEED_MAX */ 1.0 / 2.0;
  }
}
