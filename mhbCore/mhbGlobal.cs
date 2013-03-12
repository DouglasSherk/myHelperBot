using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace myHelperBot
{
  static class mhbGlobal
  {
    public static object gLock = new object();

    public static int gLeftSpeed = 0;
    public static int gRightSpeed = 0;

    public static bool gStopped = false;
  }
}
