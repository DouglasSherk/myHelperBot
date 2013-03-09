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

    private double AngleJoints(Joint a1, Joint a2, Joint b1, Joint b2, Axis ignoreAxes = Axis.None)
    {
      Vector3D aVector = new Vector3D((ignoreAxes & Axis.X) == Axis.X ? 0.0 : (a1.Position.X - a2.Position.X),
                                      (ignoreAxes & Axis.Y) == Axis.Y ? 0.0 : (a1.Position.Y - a2.Position.Y),
                                      (ignoreAxes & Axis.Z) == Axis.Z ? 0.0 : (a1.Position.Z - a2.Position.Z));
      Vector3D bVector = new Vector3D((ignoreAxes & Axis.X) == Axis.X ? 0.0 : (b1.Position.X - b2.Position.X),
                                      (ignoreAxes & Axis.Y) == Axis.Y ? 0.0 : (b1.Position.Y - b2.Position.Y),
                                      (ignoreAxes & Axis.Z) == Axis.Z ? 0.0 : (b1.Position.Z - b2.Position.Z));
      return Vector3D.AngleBetween(aVector, bVector);
    }

    private double DistanceJoints(Joint a, Joint b, Axis ignoreAxes = Axis.None)
    {
      return Math.Sqrt(((ignoreAxes & Axis.X) == Axis.X ? 0.0 : Math.Pow(a.Position.X - b.Position.X, 2.0)) +
                       ((ignoreAxes & Axis.Y) == Axis.Y ? 0.0 : Math.Pow(a.Position.Y - b.Position.Y, 2.0)) +
                       ((ignoreAxes & Axis.Z) == Axis.Z ? 0.0 : Math.Pow(a.Position.Z - b.Position.Z, 2.0)));
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

      bool isPossiblyStopGesture =
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

      if (isPossiblyStopGesture) {
        numSuccessiveStopGestures++;
      } else {
        numSuccessiveStopGestures = 0;
      }

      return numSuccessiveStopGestures >= STOP_SUCCESSIVE_GESTURES;
    }

    private bool IsInGoGesture(JointsCollection joints)
    {
      Joint leftShoulder = new Joint(),
            rightShoulder = new Joint(),
            centerShoulder = new Joint(),
            leftWrist = new Joint(),
            rightWrist = new Joint(),
            leftElbow = new Joint(),
            rightElbow = new Joint(),
            head = new Joint();

      foreach (Joint joint in joints) {
        switch (joint.ID) {
          case JointID.ElbowLeft:
            leftElbow = joint;
            break;
          case JointID.ElbowRight:
            rightElbow = joint;
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
          case JointID.ShoulderCenter:
            centerShoulder = joint;
            break;
          case JointID.Head:
            head = joint;
            break;
          default:
            break;
        }
      }

      double leftElbowAngle = AngleJoints(leftWrist, leftElbow, leftShoulder, leftElbow);
      double rightElbowAngle = AngleJoints(rightWrist, rightElbow, rightShoulder, rightElbow);
      double leftArmAngle = AngleJoints(leftWrist, leftShoulder, rightShoulder, leftShoulder, Axis.Y);
      double rightArmAngle = AngleJoints(rightWrist, rightShoulder, leftShoulder, rightShoulder, Axis.Y);
      double leftForearmAngle = AngleJoints(leftWrist, leftElbow, rightShoulder, leftShoulder, Axis.Y);
      double rightForearmAngle = AngleJoints(rightWrist, rightElbow, leftShoulder, rightShoulder, Axis.Y);
      double leftWristAngle = AngleJoints(leftWrist, leftElbow, leftShoulder, leftElbow);
      double rightWristAngle = AngleJoints(rightWrist, rightElbow, rightShoulder, rightElbow);
      double leftHandDist = DistanceJoints(leftWrist, leftShoulder, Axis.Z);
      double rightHandDist = DistanceJoints(rightWrist, rightShoulder, Axis.Z);

      if (!possibleGoGestureLeft &&
          leftWristAngle < GO_WRIST_ANGLE_START_MAX &&
          leftWristAngle > GO_WRIST_ANGLE_START_MIN) {
        lastGoGestureAngleLeftWrist = leftWristAngle + 0.001;
        lastGoGestureAngleLeftElbow = leftElbowAngle + 0.001;
        possibleGoGestureLeft = true;
      }

      if (!possibleGoGestureRight &&
          rightWristAngle < GO_WRIST_ANGLE_START_MAX &&
          rightWristAngle > GO_WRIST_ANGLE_START_MIN) {
        lastGoGestureAngleRightWrist = rightWristAngle + 0.001;
        lastGoGestureAngleRightElbow = rightElbowAngle + 0.001;
        possibleGoGestureRight = true;
      }

      if (possibleGoGestureLeft &&
          leftElbowAngle < lastGoGestureAngleLeftElbow &&
          leftWristAngle < lastGoGestureAngleLeftWrist &&
          leftArmAngle > GO_ARM_ANGLE - GO_ARM_ANGLE_TOL &&
          leftArmAngle < GO_ARM_ANGLE + GO_ARM_ANGLE_TOL &&
          (leftWrist.Position.Y > lastGoLeftWristPositionY || leftHandDist < GO_HAND_MAX_DIST)) {
        lastGoGestureAngleLeftWrist = leftWristAngle;
        lastGoGestureAngleLeftElbow = leftElbowAngle;
      } else {
        possibleGoGestureLeft = false;
      }

      if (possibleGoGestureRight &&
          ((rightElbowAngle < lastGoGestureAngleRightElbow &&
            rightWristAngle < lastGoGestureAngleRightWrist &&
            rightArmAngle > GO_ARM_ANGLE - GO_ARM_ANGLE_TOL &&
            rightArmAngle < GO_ARM_ANGLE + GO_ARM_ANGLE_TOL &&
            rightWrist.Position.Y > lastGoRightWristPositionY) ||
           rightHandDist < GO_HAND_MAX_DIST)) {
        lastGoGestureAngleRightWrist = rightWristAngle;
        lastGoGestureAngleRightElbow = rightElbowAngle;
      } else {
        possibleGoGestureRight = false;
      }

      lastGoLeftWristPositionY = leftWrist.Position.Y;
      lastGoRightWristPositionY = rightWrist.Position.Y;

      bool definitelyInGoGesture =
        (possibleGoGestureLeft &&
         lastGoGestureAngleLeftWrist <= GO_WRIST_ANGLE_END &&
         leftHandDist < GO_HAND_MAX_DIST) ||
        (possibleGoGestureRight &&
         lastGoGestureAngleRightWrist <= GO_WRIST_ANGLE_END &&
         rightHandDist < GO_HAND_MAX_DIST);

      if (definitelyInGoGesture) {
        System.Diagnostics.Debugger.Break();
        possibleGoGestureLeft = false;
        possibleGoGestureRight = false;
      }

      return definitelyInGoGesture;
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
                           IsInGoGesture(data.Joints),
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

    #region constants
    private enum Axis {
      None = 0,
      X = 1,
      Y = 2,
      Z = 4,
    };

    private const int STOP_SUCCESSIVE_GESTURES = 3;

    private const double STOP_ARM_ANGLE = 90.0;
    private const double STOP_ARM_ANGLE_TOL = 20.0;
    private const double STOP_HAND_ANGLE = 35.0;
    private const double STOP_HAND_ANGLE_TOL = 15.0;

    private const double GO_WRIST_ANGLE_START_MIN = 140.0;
    private const double GO_WRIST_ANGLE_START_MAX = 180.0;
    private const double GO_WRIST_ANGLE_END = 110.0;
    private const double GO_ARM_ANGLE = 90.0;
    private const double GO_ARM_ANGLE_TOL = 25.0;
    private const double GO_HAND_MAX_DIST = 0.2;
    #endregion constants

    #region Private state
    private KinectNui.Runtime _Kinect;
    private DateTime lastRequest;

    private int numSuccessiveStopGestures = 0;

    private bool possibleGoGestureLeft = false;
    private bool possibleGoGestureRight = false;
    private double lastGoGestureAngleLeftElbow;
    private double lastGoGestureAngleRightElbow;
    private double lastGoGestureAngleLeftWrist;
    private double lastGoGestureAngleRightWrist;
    private double lastGoLeftWristPositionY;
    private double lastGoRightWristPositionY;
    #endregion Private state
  }
}
