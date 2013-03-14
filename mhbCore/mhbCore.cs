//#define DEBUG_THREADS
#define DEBUG_GESTURES
#define DEBUG_TRACKING

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows.Media.Media3D;
using Microsoft.Research.Kinect.Nui;
using KinectNui = Microsoft.Research.Kinect.Nui;

namespace myHelperBot
{
  class mhbCore
  {
    public mhbCore()
    {
      mLogicThread = new Thread(new ThreadStart(mLogic.loop));
      mLogicThread.Name = "LogicThread";
      mLogicThread.Start();

      mSerialThread = new Thread(new ThreadStart(mSerial.loop));
      mSerialThread.Name = "SerialThread";
      mSerialThread.Start();

      mSoundThread = new Thread(new ThreadStart(mSound.loop));
      mSoundThread.Name = "SoundThread";
      mSoundThread.Start();
    }

    public void SetKinect(KinectNui.Runtime kinect)
    {
      if (mKinectThread != null) {
        mKinectThread.Abort();
        mKinectThread = null;
      }

      mKinect.Set(kinect);
      mKinectThread = new Thread(new ThreadStart(mKinect.Init));
      mKinectThread.Name = "KinectThread";
      mKinectThread.Start();
    }

    public bool HasKinect()
    {
      return mKinect._Kinect != null;
    }

    public static void DebugThread(string message)
    {
#if DEBUG_THREADS
      Console.WriteLine(Thread.CurrentThread.Name + ": " + message);
#endif
    }

    public static void DebugGesture(string message)
    {
#if DEBUG_GESTURES
      Console.WriteLine(message);
      //Console.Beep();
#endif
    }

    public static void DebugTracking(Point3D position, int leftSpeed, int rightSpeed, string message)
    {
#if DEBUG_TRACKING
      Console.WriteLine(message + " %(" + leftSpeed + ", " + rightSpeed + ") @(" +
        Math.Round(position.X, 2) + ", " + Math.Round(position.Y, 2) + ", " + Math.Round(position.Z, 2) + ")");
#endif
    }

    public static void DebugPrint(string message)
    {
#if DEBUG
      Console.WriteLine(message);
#endif
    }

    #region state
    private mhbKinect mKinect = new mhbKinect();
    private mhbSerial mSerial = new mhbSerial();
    private mhbLogic mLogic = new mhbLogic();
    private mhbSound mSound = new mhbSound();

    private Thread mSerialThread = null;
    private Thread mKinectThread = null;
    private Thread mLogicThread = null;
    private Thread mSoundThread = null;
    #endregion
  }
}
