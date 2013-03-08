using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
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
    private Joint FindJoint(JointsCollection joints, JointID jointID)
    {
      foreach (Joint joint in joints) {
        if (joint.ID == jointID) {
          return joint;
        }
      }

      return new Joint();
    }

    private double AngleJoints(Joint a1, Joint a2, Joint b1, Joint b2)
    {
      Vector3D aVector = new Vector3D(a1.Position.X - a2.Position.X,
                                      a1.Position.Y - a2.Position.Y,
                                      a1.Position.Z - a2.Position.Z);
      Vector3D bVector = new Vector3D(b1.Position.X - b2.Position.X,
                                      b1.Position.Y - b2.Position.Y,
                                      b1.Position.Z - b2.Position.Z);
      return Vector3D.AngleBetween(aVector, bVector);
    }

    private bool IsInStopGesture(JointsCollection joints)
    {
      Joint head = new Joint(),
            spine = new Joint(),
            leftShoulder = new Joint(),
            rightShoulder = new Joint(),
            leftWrist = new Joint(),
            rightWrist = new Joint(),
            leftHand = new Joint(),
            rightHand = new Joint();

      foreach (Joint joint in joints) {
        switch (joint.ID) {
          case JointID.Head:
            head = joint;
            break;
          case JointID.Spine:
            spine = joint;
            break;
          case JointID.HandLeft:
            leftHand = joint;
            break;
          case JointID.HandRight:
            rightHand = joint;
            break;
          case JointID.WristLeft:
            leftWrist = joint;
            break;
          case JointID.WristRight:
            rightWrist = joint;
            break;
          case JointID.ShoulderLeft:
            leftShoulder = joint;
            break;
          case JointID.ShoulderRight:
            rightShoulder = joint;
            break;
          default:
            break;
        }
      }

      double leftArmAngle = AngleJoints(leftShoulder, leftWrist, head, spine);
      double rightArmAngle = AngleJoints(rightShoulder, rightWrist, head, spine);
      double leftHandAngle = AngleJoints(leftHand, leftWrist, leftWrist, leftShoulder);
      double rightHandAngle = AngleJoints(rightHand, rightWrist, rightWrist, rightShoulder);

      return
        (leftArmAngle > STOP_ARM_ANGLE - STOP_ARM_ANGLE_TOL &&
         leftArmAngle < STOP_ARM_ANGLE + STOP_ARM_ANGLE_TOL &&
         leftHandAngle > STOP_HAND_ANGLE - STOP_HAND_ANGLE_TOL &&
         leftHandAngle < STOP_HAND_ANGLE + STOP_HAND_ANGLE_TOL &&
         leftHand.Position.Y > leftWrist.Position.Y &&
         leftShoulder.Position.Z > leftWrist.Position.Z) ||
        (rightArmAngle > STOP_ARM_ANGLE - STOP_ARM_ANGLE_TOL &&
         rightArmAngle < STOP_ARM_ANGLE + STOP_ARM_ANGLE_TOL &&
         rightHandAngle > STOP_HAND_ANGLE - STOP_HAND_ANGLE_TOL &&
         rightHandAngle < STOP_HAND_ANGLE + STOP_HAND_ANGLE_TOL &&
         rightHand.Position.Y > rightWrist.Position.Y &&
         rightShoulder.Position.Z > rightWrist.Position.Z);
    }

    private void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e) {
      SkeletonFrame skeletonFrame = e.SkeletonFrame;

      //KinectSDK TODO: this shouldn't be needed, but if power is removed from the Kinect, you may still get an event here, but skeletonFrame will be null.
      if (skeletonFrame == null) {
        return;
      }

      foreach (SkeletonData data in skeletonFrame.Skeletons) {
        if (SkeletonTrackingState.Tracked == data.TrackingState) {
          WriteHttpRequest(FindJoint(data.Joints, JointID.Spine),
                           IsInStopGesture(data.Joints),
                           false,
                           false);
          break;
        }
      }
    }
    #endregion Skeletal processing

    #region http
    private void WriteHttpRequest(Joint joint, bool gestureStop, bool gestureGo, bool gestureSave) {
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
        //client.DownloadString(request);
        lastRequest = DateTime.Now;
      }
    }
    #endregion http

    #region Private state
    private KinectNui.Runtime _Kinect;
    private DateTime lastRequest;

    private const double STOP_ARM_ANGLE = 90.0;
    private const double STOP_ARM_ANGLE_TOL = 20.0;
    private const double STOP_HAND_ANGLE = 35.0;
    private const double STOP_HAND_ANGLE_TOL = 15.0;

    private const double MIN_STOP_DIST = 0.15;
    private const double MIN_GO_DIST = 0.10;
    private const double MIN_SAVE_DIST = 0.30;
    #endregion Private state
  }
}
