using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.Research.Kinect.Nui;
using KinectNui = Microsoft.Research.Kinect.Nui;

namespace myHelperBot
{
  class mhbCore
  {
    public mhbCore()
    {
      mSerialThread = new Thread(new ThreadStart(mSerial.loop));
      mSerialThread.Start();

      mLogicThread = new Thread(new ThreadStart(mLogic.loop));
      mLogicThread.Start();
    }

    public void SetKinect(KinectNui.Runtime kinect)
    {
      if (mKinectThread != null) {
        mKinectThread.Abort();
        mKinectThread = null;
      }

      mKinect.Set(kinect);
      mKinectThread = new Thread(new ThreadStart(mKinect.Init));
      mKinectThread.Start();
    }

    #region state
    private mhbKinect mKinect = new mhbKinect();
    private mhbSerial mSerial = new mhbSerial();
    private mhbLogic mLogic = new mhbLogic();

    private Thread mSerialThread = null;
    private Thread mKinectThread = null;
    private Thread mLogicThread = null;
    #endregion
  }
}
