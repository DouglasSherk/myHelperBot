using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace myHelperBot
{
  class mhbState
  {
    public static mhbState g = new mhbState();

    public static object Lock = new object();

    public bool isTracking = false;

    public int leftSpeed = 0;
    public int rightSpeed = 0;

    public bool stopped = false;

    public Point3D userPosition = new Point3D();

    public bool isInStopGesture = false;
    public bool isInGoGesture = false;
    public bool isInSaveGesture = false;
    public bool isInRelocateGesture = false;
  }
}
