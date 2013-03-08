using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using System.Windows;
using Microsoft.Research.Kinect.Nui;
using KinectNui = Microsoft.Research.Kinect.Nui;

namespace myHelperBot
{
  class mhbLogic
  {
    #region Initialization
    public mhbLogic()
    {
        //InitializeComponent();
      lastRequest = DateTime.Now;
    }

    public RuntimeOptions RuntimeOptions { get; private set; }

    private void InitRuntime() {
      //Some Runtimes' status will be NotPowered, or some other error state. Only want to Initialize the runtime, if it is connected.
      if (_Kinect.Status == KinectStatus.Connected) {
        bool skeletalViewerAvailable = true; // IsSkeletalViewerAvailable;

        // NOTE:  Skeletal tracking only works on one Kinect per process right now.
        RuntimeOptions = skeletalViewerAvailable ?
                             RuntimeOptions.UseDepthAndPlayerIndex | RuntimeOptions.UseSkeletalTracking | RuntimeOptions.UseColor
                             : RuntimeOptions.UseDepth | RuntimeOptions.UseColor;
        _Kinect.Initialize(RuntimeOptions);
        //skeletonPanel.Visibility = skeletalViewerAvailable ? System.Windows.Visibility.Visible : System.Windows.Visibility.Collapsed;
        if (RuntimeOptions.HasFlag(RuntimeOptions.UseSkeletalTracking)) {
          _Kinect.SkeletonEngine.TransformSmooth = true;
        }
      }
    }

    public void ReInitRuntime()
    {
        // Will call Uninitialize followed by Initialize.
        this.Kinect = this.Kinect;
    }

    public KinectNui.Runtime Kinect {
      get { return _Kinect; }
      set {
        //Clean up existing runtime if we are being set to null, or a new Runtime.
        if (_Kinect != null) {
          _Kinect.SkeletonFrameReady -= new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
          _Kinect.Uninitialize();
        }

        _Kinect = value;

        if (_Kinect != null) {
          InitRuntime();
          _Kinect.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
        }
      }
    }
    #endregion Initialization

    #region Skeletal processing
    private void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e) {
      SkeletonFrame skeletonFrame = e.SkeletonFrame;

      //KinectSDK TODO: this shouldn't be needed, but if power is removed from the Kinect, you may still get an event here, but skeletonFrame will be null.
      if (skeletonFrame == null) {
        return;
      }
      
      bool found = false;

      bool gestureStop = false;
      bool gestureGo = false;
      bool gestureSaveLeft = false, gestureSaveRight = false;

      Joint spine = new Joint();
      foreach (SkeletonData data in skeletonFrame.Skeletons) {
        if (SkeletonTrackingState.Tracked == data.TrackingState) {
          foreach (Joint joint in data.Joints) {
            // Check if we're tracking a reasonable spot on the skeleton (spine).
            if (joint.ID == JointID.Spine) {
              found = true;
              spine = joint;
            }

            // Check if the left or right hand joints are closer to the camera than any other joint.
            if ((joint.ID == JointID.HandLeft || joint.ID == JointID.HandRight) && !gestureStop) {
              gestureStop = gestureGo = true;

              // Gesture stop.
              foreach (Joint cmpHandJoint in data.Joints) {
                if (cmpHandJoint.ID != JointID.HandLeft && cmpHandJoint.ID != JointID.HandRight &&
                    cmpHandJoint.ID != JointID.WristLeft && cmpHandJoint.ID != JointID.WristRight &&
                    cmpHandJoint.ID != JointID.ElbowLeft && cmpHandJoint.ID != JointID.ElbowRight &&
                    joint.Position.Z > cmpHandJoint.Position.Z - MIN_STOP_DIST) {
                  gestureStop = false;
                  break;
                }
              }
              // Gesture go.
              foreach (Joint cmpHandJoint in data.Joints) {
                if (cmpHandJoint.ID != JointID.HandLeft && cmpHandJoint.ID != JointID.HandRight &&
                    cmpHandJoint.ID != JointID.WristLeft && cmpHandJoint.ID != JointID.WristRight &&
                    cmpHandJoint.ID != JointID.ElbowLeft && cmpHandJoint.ID != JointID.ElbowRight &&
                    joint.Position.Y < cmpHandJoint.Position.Y + MIN_GO_DIST) {
                  gestureGo = false;
                  break;
                }
              }
            }

            bool blah;
            // Save gesture (hands on shoulders)
            if ((joint.ID == JointID.HandLeft && !gestureSaveLeft) ||
                (joint.ID == JointID.HandRight && !gestureSaveRight)) {
              blah = joint.ID == JointID.HandLeft ? (gestureSaveLeft = true) : (gestureSaveRight = true);
              foreach (Joint cmpHandJoint in data.Joints) {
                if (((cmpHandJoint.ID == JointID.ShoulderLeft && joint.ID == JointID.HandRight) || 
                     (cmpHandJoint.ID == JointID.ShoulderRight && joint.ID == JointID.HandLeft)) &&
                    (Math.Abs(joint.Position.Z - cmpHandJoint.Position.Z) > MIN_SAVE_DIST ||
                     Math.Abs(joint.Position.Y - cmpHandJoint.Position.Y) > MIN_SAVE_DIST ||
                     Math.Abs(joint.Position.X - cmpHandJoint.Position.X) > MIN_SAVE_DIST)) {
                  blah = joint.ID == JointID.HandLeft ? (gestureSaveLeft = false) : (gestureSaveRight = false);
                  break;
                }
              }
            }
          }
        }

        if (found) {
          writeHttpRequest(spine, gestureStop, gestureGo, gestureSaveLeft && gestureSaveRight);
          break;
        }
      }
    }
    #endregion Skeletal processing

    #region http
    private void writeHttpRequest(Joint joint, bool gestureStop, bool gestureGo, bool gestureSave) {
      if ((DateTime.Now - lastRequest).TotalMilliseconds > 25) {
        WebClient client = new WebClient();
        string request = "http://192.168.137.183/?data=" +
                         Convert.ToInt32(gestureStop) + ", " +
                         Convert.ToInt32(gestureGo) + ", " +
                         Convert.ToInt32(gestureSave) + ", " +
                         joint.Position.X + ", " +
                         joint.Position.Y + ", " +
                         joint.Position.Z;
        Console.WriteLine(request);
        Console.ReadLine();
        client.DownloadString(request);
        lastRequest = DateTime.Now;
      }
    }
    #endregion http

    #region Private state
    private KinectNui.Runtime _Kinect;
    private DateTime lastRequest;

    private const double MIN_STOP_DIST = 0.15;
    private const double MIN_GO_DIST = 0.10;
    private const double MIN_SAVE_DIST = 0.30;
    #endregion Private state
  }
}
