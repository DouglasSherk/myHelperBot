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
            centerShoulder = new Joint(),
            leftWrist = new Joint(),
            rightWrist = new Joint(),
            leftHand = new Joint(),
            rightHand = new Joint(),
            leftElbow = new Joint(),
            rightElbow = new Joint();

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
          case JointID.ElbowLeft:
            leftElbow = joint;
            break;
          case JointID.ElbowRight:
            rightElbow = joint;
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
          default:
            break;
        }
      }

      double leftElbowAngle = AngleJoints(leftWrist, leftElbow, leftShoulder, leftElbow);
      double rightElbowAngle = AngleJoints(rightWrist, rightElbow, rightShoulder, rightElbow);
      double handDist = DistanceJoints(leftHand, rightHand);

      bool isPossiblyStopGesture =
        Math.Abs(leftElbowAngle - rightElbowAngle) < STOP_ELBOW_ANGLE_TOL &&
        handDist >= STOP_HAND_MIN_DIST &&
        leftHand.Position.Y > centerShoulder.Position.Y &&
        rightHand.Position.Y > centerShoulder.Position.Y &&
        leftHand.Position.Y < head.Position.Y &&
        rightHand.Position.Y < head.Position.Y;

      if (isPossiblyStopGesture) {
        numSuccessiveStopGestures++;
      } else {
        numSuccessiveStopGestures = 0;
      }

      bool definitelyInStopGesture = numSuccessiveStopGestures >= STOP_SUCCESSIVE_GESTURES;

      if (definitelyInStopGesture) {
        //System.Diagnostics.Debugger.Break();
        possibleGoGesture = false;
        numSuccessiveStopGestures = 0;
      }

      return definitelyInStopGesture;
    }

    private bool IsInGoGesture(JointsCollection joints)
    {
      if (numSuccessiveStopGestures > 0) {
        return false;
      }

      Joint leftHand = new Joint(),
            rightHand = new Joint(),
            leftWrist = new Joint(),
            rightWrist = new Joint(),
            leftElbow = new Joint(),
            rightElbow = new Joint(),
            spine = new Joint(),
            head = new Joint();

      foreach (Joint joint in joints) {
        switch (joint.ID) {
          case JointID.HandLeft:
            leftHand = joint;
            break;
          case JointID.HandRight:
            rightHand = joint;
            break;
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
          case JointID.Head:
            head = joint;
            break;
          case JointID.Spine:
            spine = joint;
            break;
          default:
            break;
        }
      }

      double leftWristAngle = AngleJoints(leftHand, leftWrist, leftElbow, leftWrist);
      double rightWristAngle = AngleJoints(rightHand, rightWrist, rightElbow, rightWrist);
      double handDist = DistanceJoints(leftHand, rightHand);

      if (leftHand.Position.Y < head.Position.Y && rightHand.Position.Y < head.Position.Y &&
          leftHand.Position.Y > spine.Position.Y && rightHand.Position.Y > spine.Position.Y &&
          leftWristAngle >= GO_WRIST_START_ANGLE && rightWristAngle >= GO_WRIST_START_ANGLE &&
          handDist < GO_HAND_MAX_DIST && !possibleGoGesture) {
        possibleGoGesture = true;
        lastGoLeftWristAngle = leftWristAngle;
        lastGoRightWristAngle = rightWristAngle;
        startGoLeftHandY = leftHand.Position.Y;
        startGoRightHandY = rightHand.Position.Y;
        startGoLeftHandZ = leftHand.Position.Z;
        startGoRightHandZ = rightHand.Position.Z;
        startGoTime = DateTime.Now;
      }

      bool definitelyInGoGesture = false;

      if (possibleGoGesture && (DateTime.Now - startGoTime).TotalMilliseconds >= GO_INTERVAL) {
        if (leftWristAngle < lastGoLeftWristAngle && rightWristAngle < lastGoRightWristAngle &&
            (leftHand.Position.Y > startGoLeftHandY && rightHand.Position.Y > startGoRightHandY ||
             leftHand.Position.Z > startGoLeftHandZ && rightHand.Position.Z > startGoRightHandZ)) {
          definitelyInGoGesture = true;
          //System.Diagnostics.Debugger.Break();
        }

        possibleGoGesture = false;
      }
      
      return definitelyInGoGesture;
    }

    private bool IsInSaveGesture(JointsCollection joints)
    {
      Joint leftHand = new Joint(),
      rightHand = new Joint(),
      spine = new Joint(),
      centerShoulder = new Joint();

      foreach (Joint joint in joints) {
        switch (joint.ID) {
          case JointID.HandLeft:
            leftHand = joint;
            break;
          case JointID.HandRight:
            rightHand = joint;
            break;
          case JointID.Spine:
            spine = joint;
            break;
          case JointID.ShoulderCenter:
            centerShoulder = joint;
            break;
          default:
            break;
        }
      }

      double handDist = DistanceJoints(leftHand, rightHand);

      if (!possibleSaveGesture) {
        if (leftHand.Position.Y < centerShoulder.Position.Y &&
            rightHand.Position.Y < centerShoulder.Position.Y &&
            handDist >= SAVE_HAND_START_DIST) {
          possibleSaveGesture = true;
          startSaveTime = DateTime.Now;
        }
      }

      bool definitelyInSaveGesture = false;

      if (possibleSaveGesture && (DateTime.Now - startSaveTime).TotalMilliseconds >= SAVE_INTERVAL) {
        if (leftHand.Position.Y < spine.Position.Y &&
            rightHand.Position.Y < spine.Position.Y &&
            handDist <= SAVE_HAND_END_DIST) {
          definitelyInSaveGesture = true;
          //System.Diagnostics.Debugger.Break();
        }

        possibleSaveGesture = false;
      }

      return definitelyInSaveGesture;
    }

    private bool IsInRelocateGesture(JointsCollection joints)
    {
      Joint leftHand = new Joint(),
            rightHand = new Joint(),
            leftShoulder = new Joint(),
            rightShoulder = new Joint();

      foreach (Joint joint in joints) {
        switch (joint.ID) {
          case JointID.HandLeft:
            leftHand = joint;
            break;
          case JointID.HandRight:
            rightHand = joint;
            break;
          case JointID.ShoulderLeft:
            leftShoulder = joint;
            break;
          case JointID.ShoulderRight:
            rightShoulder = joint;
            break;
        }
      }

      double handDist = DistanceJoints(leftHand, rightHand);

      if (handDist <= RELOCATE_HAND_MAX_DIST &&
          !possibleRelocateGestureLeftward && !possibleRelocateGestureRightward) {
        if (!possibleRelocateGestureLeftward && leftHand.Position.X < leftShoulder.Position.X) {
          possibleRelocateGestureLeftward = true;
          startRelocateTime = DateTime.Now;
        } else if (!possibleRelocateGestureRightward && rightHand.Position.X > rightShoulder.Position.X) {
          possibleRelocateGestureRightward = true;
          startRelocateTime = DateTime.Now;
        }
      }

      bool definitelyInRelocateGesture = false;

      if ((possibleRelocateGestureLeftward || possibleRelocateGestureRightward) &&
          (DateTime.Now - startRelocateTime).TotalMilliseconds >= RELOCATE_INTERVAL) {
        if (handDist <= RELOCATE_HAND_MAX_DIST && 
            (possibleRelocateGestureLeftward && rightHand.Position.X > rightShoulder.Position.X) ||
            (possibleRelocateGestureRightward && leftHand.Position.X < leftShoulder.Position.X)) {
          definitelyInRelocateGesture = true;
          //System.Diagnostics.Debugger.Break();
        }

        possibleRelocateGestureLeftward = possibleRelocateGestureRightward = false;
      }

      return definitelyInRelocateGesture;
    }

    private void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e) {
      SkeletonFrame skeletonFrame = e.SkeletonFrame;

      //KinectSDK TODO: this shouldn't be needed, but if power is removed from the Kinect, you may still get an event here, but skeletonFrame will be null.
      if (skeletonFrame == null) {
        return;
      }

      foreach (SkeletonData data in skeletonFrame.Skeletons) {
        if (SkeletonTrackingState.NotTracked != data.TrackingState) {
          WriteHttpRequest(FindJoint(data.Joints, JointID.Spine),
                           IsInStopGesture(data.Joints),
                           IsInGoGesture(data.Joints),
                           IsInSaveGesture(data.Joints),
                           IsInRelocateGesture(data.Joints));
          break;
        }
      }
    }
    #endregion Skeletal processing

    #region http
    private void WriteHttpRequest(Joint joint, bool gestureStop, bool gestureGo, bool gestureSave, bool gestureRelocate) {
      if ((DateTime.Now - lastRequest).TotalMilliseconds > 10) {
        string request = "http://192.168.137.183/?data=" +
                         Convert.ToInt32(gestureStop) + ", " +
                         Convert.ToInt32(gestureGo) + ", " +
                         Convert.ToInt32(gestureSave) + ", " +
                         //Convert.ToInt32(gestureRelocate) + ", " +
                         joint.Position.X + ", " +
                         joint.Position.Y + ", " +
                         joint.Position.Z;
        try {
          webClient.DownloadString(request);
        } catch {

        }
        Console.WriteLine(request);
        Console.ReadLine();
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

    private const double STOP_ELBOW_ANGLE_TOL = 10.0;
    private const double STOP_HAND_MIN_DIST = 0.5;

    private const double GO_HAND_MAX_DIST = 0.4;
    private const double GO_WRIST_START_ANGLE = 165.0;
    private const double GO_WRIST_END_ANGLE = 145.0;
    private const int GO_INTERVAL = 250;

    private const double SAVE_HAND_START_DIST = 0.5;
    private const double SAVE_HAND_END_DIST = 0.2;
    private const int SAVE_INTERVAL = 750;

    private const double RELOCATE_HAND_MAX_DIST = 0.3;
    private const int RELOCATE_INTERVAL = 500;
    #endregion constants

    #region Private state
    private KinectNui.Runtime _Kinect;
    private DateTime lastRequest;
    private WebClient webClient = new WebClient();

    private int numSuccessiveStopGestures = 0;

    private bool possibleGoGesture = false;
    private double lastGoLeftWristAngle;
    private double lastGoRightWristAngle;
    private double startGoLeftHandY;
    private double startGoRightHandY;
    private double startGoLeftHandZ;
    private double startGoRightHandZ;
    private DateTime startGoTime;

    private bool possibleSaveGesture = false;
    private DateTime startSaveTime;

    private bool possibleRelocateGestureLeftward = false;
    private bool possibleRelocateGestureRightward = false;
    private DateTime startRelocateTime;
    #endregion Private state
  }
}
