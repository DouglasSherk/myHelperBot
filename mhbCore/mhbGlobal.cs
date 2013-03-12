using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace myHelperBot
{
  static class mhbGlobal
  {
    public static object gLock = new object();

    public static bool gIsTracking = false;

    public static int gLeftSpeed = 0;
    public static int gRightSpeed = 0;

    public static bool gStopped = false;

    public static Point3D gUserPosition = new Point3D();

    public static bool gIsInStopGesture = false;
    public static bool gIsInGoGesture = false;
    public static bool gIsInSaveGesture = false;
    public static bool gIsInRelocateGesture = false;
  }
}
