using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows.Media.Media3D;
using System.Net;
using System.Text;
using System.Threading;
using System.Windows;
using Microsoft.Kinect;
using Microsoft.Speech;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;

namespace myHelperBot
{
  [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1001:TypesThatOwnDisposableFieldsShouldBeDisposable",
    Justification = "In a full-fledged application, the SpeechRecognitionEngine object should be properly disposed. For the sake of simplicity, we're omitting that code in this sample.")]
  class mhbKinect
  {
    #region Initialization
    public mhbKinect()
    {
      
    }

    ~mhbKinect()
    {
      if (mKinect != null) {
        mKinect.Stop();
      }
    }

    private static RecognizerInfo GetKinectRecognizer() {
      foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers()) {
        string value;
        recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
        if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase)) {
          return recognizer;
        }
      }

      return null;
    }

    public void InitKinect()
    {
      foreach (var potentialSensor in KinectSensor.KinectSensors) {
        if (potentialSensor.Status == KinectStatus.Connected) {
          this.mKinect = potentialSensor;
          break;
        }
      }

      if (this.mKinect != null) {
        // Turn on the skeleton stream to receive skeleton frames
        this.mKinect.SkeletonStream.Enable();

        // Add an event handler to be called whenever there is new color frame data
        this.mKinect.SkeletonFrameReady += this.SensorSkeletonFrameReady;

        // Start the sensor!
        try {
          this.mKinect.Start();
        } catch {
          this.mKinect = null;
        }
      }

      RecognizerInfo ri = GetKinectRecognizer();
      if (ri != null) {
        this.mSpeechRecognizer = new SpeechRecognitionEngine(ri.Id);

        var directions = new Choices();
        directions.Add(new SemanticResultValue("go", "GO"));
        directions.Add(new SemanticResultValue("stop", "STOP"));
        directions.Add(new SemanticResultValue("save", "SAVE"));
        directions.Add(new SemanticResultValue("relocate", "RELOCATE"));

        var gb = new GrammarBuilder { Culture = ri.Culture };
        gb.Append(directions);

        var g = new Grammar(gb);

        this.mSpeechRecognizer.LoadGrammar(g);
      }

      mSpeechRecognizer.SpeechRecognized += SpeechRecognized;
      mSpeechRecognizer.SpeechRecognitionRejected += SpeechRejected;

      mSpeechRecognizer.SetInputToAudioStream(
        mKinect.AudioSource.Start(), new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
      mSpeechRecognizer.RecognizeAsync(RecognizeMode.Multiple);
    }

    public void DestroyKinect()
    {
      if (this.mKinect != null) {
        this.mKinect.AudioSource.Stop();

        this.mKinect.Stop();
        this.mKinect = null;
      }

      if (this.mSpeechRecognizer != null) {
        this.mSpeechRecognizer.SpeechRecognized -= SpeechRecognized;
        this.mSpeechRecognizer.SpeechRecognitionRejected -= SpeechRejected;
        this.mSpeechRecognizer.RecognizeAsyncStop();
      }
    }
    #endregion Initialization

    #region speech processing
    private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
    {
      // Speech utterance confidence below which we treat speech as if it hadn't been heard
      const double ConfidenceThreshold = 0.3;

      if (e.Result.Confidence >= ConfidenceThreshold) {
        switch (e.Result.Semantics.Value.ToString()) {
          case "GO":
            lock (mhbState.Lock) {
              mhbState.g.isInGoGesture = true;
            }
            break;

          case "STOP":
            lock (mhbState.Lock) {
              mhbState.g.isInStopGesture = true;
            }
            break;

          case "SAVE":
            lock (mhbState.Lock) {
              mhbState.g.isInSaveGesture = true;
            }
            break;

          case "RELOCATE":
            lock (mhbState.Lock) {
              mhbState.g.isInRelocateGesture = true;
            }
            break;
        }
      }
    }

    private void SpeechRejected(object sender, SpeechRecognitionRejectedEventArgs e)
    {

    }
    #endregion speech processing

    #region Utils
    private bool InRange(double val, double min, double max)
    {
      return val >= min && val <= max;
    }
    #endregion Utils

    #region Skeletal processing
    private Joint FindJoint(JointCollection joints, JointType JointType)
    {
      foreach (Joint joint in joints) {
        if (joint.JointType == JointType) {
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

    private bool IsInStopGesture(Skeleton data)
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

      foreach (Joint joint in data.Joints) {
        bool lowQuality = joint.TrackingState == JointTrackingState.NotTracked;

        switch (joint.JointType) {
          case JointType.Head:
            head = joint;
            break;
          case JointType.Spine:
            spine = joint;
            break;
          case JointType.HandLeft:
            leftHand = joint;
            break;
          case JointType.HandRight:
            rightHand = joint;
            break;
          case JointType.WristLeft:
            leftWrist = joint;
            break;
          case JointType.WristRight:
            rightWrist = joint;
            break;
          case JointType.ElbowLeft:
            leftElbow = joint;
            break;
          case JointType.ElbowRight:
            rightElbow = joint;
            break;
          case JointType.ShoulderLeft:
            leftShoulder = joint;
            break;
          case JointType.ShoulderRight:
            rightShoulder = joint;
            break;
          case JointType.ShoulderCenter:
            centerShoulder = joint;
            break;
          default:
            lowQuality = false;
            break;
        }

        if (lowQuality) {
          return false;
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
        possibleGoGesture = false;
        numSuccessiveStopGestures = 0;
        mhbCore.DebugGesture("Stop");
      }

      return definitelyInStopGesture;
    }

    private bool IsInGoGesture(Skeleton data)
    {
      Joint leftHand = new Joint(),
            rightHand = new Joint(),
            leftShoulder = new Joint(),
            rightShoulder = new Joint();

      foreach (Joint joint in data.Joints) {
        bool lowQuality = joint.TrackingState == JointTrackingState.NotTracked;

        switch (joint.JointType) {
          case JointType.HandLeft:
            leftHand = joint;
            break;
          case JointType.HandRight:
            rightHand = joint;
            break;
          case JointType.ShoulderLeft:
            leftShoulder = joint;
            break;
          case JointType.ShoulderRight:
            rightShoulder = joint;
            break;
          default:
            lowQuality = false;
            break;
        }

        if (lowQuality) {
          return false;
        }
      }

      double handDist = DistanceJoints(leftHand, rightHand);
      double leftHandDist = DistanceJoints(leftHand, leftShoulder);
      double rightHandDist = DistanceJoints(rightHand, rightShoulder);

      if (!possibleGoGesture &&
          InRange(handDist, GO_HAND_MIN_DIFF, GO_HAND_MAX_DIFF) &&
          Math.Abs(leftHandDist - rightHandDist) <= GO_HAND_MAX_DIFF &&
          InRange(leftHandDist, GO_HAND_START_MIN_DIST, GO_HAND_START_MAX_DIST) &&
          InRange(rightHandDist, GO_HAND_START_MIN_DIST, GO_HAND_START_MAX_DIST) &&
          leftHand.Position.Z < leftShoulder.Position.Z &&
          rightHand.Position.Z < rightShoulder.Position.Z) {
        possibleGoGesture = true;
        startGoTime = DateTime.Now;
      }

      bool definitelyInGoGesture = false;

      if (possibleGoGesture &&
          InRange(handDist, GO_HAND_MIN_DIFF, GO_HAND_MAX_DIFF) &&
          Math.Abs(leftHandDist - rightHandDist) <= GO_HAND_MAX_DIFF &&
          InRange(leftHandDist, GO_HAND_END_MIN_DIST, GO_HAND_END_MAX_DIST) &&
          InRange(rightHandDist, GO_HAND_END_MIN_DIST, GO_HAND_END_MAX_DIST) &&
          leftHand.Position.Z < leftShoulder.Position.Z &&
          rightHand.Position.Z < rightShoulder.Position.Z) {
        definitelyInGoGesture = true;
        possibleGoGesture = false;
        mhbCore.DebugGesture("Go");
      }

      if (possibleGoGesture &&
          (DateTime.Now - startGoTime).TotalMilliseconds >= GO_INTERVAL) {
        possibleGoGesture = false;
      }
      
      return definitelyInGoGesture;
    }

    private bool IsInSaveGesture(Skeleton data)
    {
      Joint leftHand = new Joint(),
      rightHand = new Joint(),
      spine = new Joint(),
      centerShoulder = new Joint();

      foreach (Joint joint in data.Joints) {
        bool lowQuality = joint.TrackingState == JointTrackingState.NotTracked;

        switch (joint.JointType) {
          case JointType.HandLeft:
            leftHand = joint;
            break;
          case JointType.HandRight:
            rightHand = joint;
            break;
          case JointType.Spine:
            spine = joint;
            break;
          case JointType.ShoulderCenter:
            centerShoulder = joint;
            break;
          default:
            lowQuality = false;
            break;
        }

        if (lowQuality) {
          return false;
        }
      }

      double handDist = DistanceJoints(leftHand, rightHand);

      if (!possibleSaveGesture &&
          leftHand.Position.Y < centerShoulder.Position.Y &&
          rightHand.Position.Y < centerShoulder.Position.Y &&
          leftHand.Position.Y > spine.Position.Y &&
          rightHand.Position.Y > spine.Position.Y &&
          handDist >= SAVE_HAND_START_DIST) {
        possibleSaveGesture = true;
        startSaveTime = DateTime.Now;
      }

      bool definitelyInSaveGesture = false;

      if (possibleSaveGesture &&
          leftHand.Position.Y < spine.Position.Y &&
          rightHand.Position.Y < spine.Position.Y &&
          handDist <= SAVE_HAND_END_DIST) {
        definitelyInSaveGesture = true;
        possibleSaveGesture = false;
        mhbCore.DebugGesture("Save");
      }

      if (possibleSaveGesture &&
          (DateTime.Now - startSaveTime).TotalMilliseconds >= SAVE_INTERVAL) {
        possibleSaveGesture = false;
      }

      return definitelyInSaveGesture;
    }

    private bool IsInRelocateGesture(Skeleton data)
    {
      if (possibleGoGesture) {
        return false;
      }

      Joint leftHand = new Joint(),
            rightHand = new Joint(),
            leftShoulder = new Joint(),
            rightShoulder = new Joint(),
            spine = new Joint();

      foreach (Joint joint in data.Joints) {
        bool lowQuality = joint.TrackingState == JointTrackingState.NotTracked;

        switch (joint.JointType) {
          case JointType.HandLeft:
            leftHand = joint;
            break;
          case JointType.HandRight:
            rightHand = joint;
            break;
          case JointType.ShoulderLeft:
            leftShoulder = joint;
            break;
          case JointType.ShoulderRight:
            rightShoulder = joint;
            break;
          case JointType.Spine:
            spine = joint;
            break;
          default:
            lowQuality = false;
            break;
        }

        if (lowQuality) {
          return false;
        }
      }

      double handDist = DistanceJoints(leftHand, rightHand);
      double leftShoulderDist = DistanceJoints(leftHand, leftShoulder);
      double rightShoulderDist = DistanceJoints(rightHand, rightShoulder);

      if (handDist <= RELOCATE_HAND_MAX_DIST &&
          leftHand.Position.Y > spine.Position.Y && rightHand.Position.Y > spine.Position.Y &&
          leftHand.Position.Y < leftShoulder.Position.Y && rightHand.Position.Y < rightShoulder.Position.Y &&
          leftShoulderDist >= RELOCATE_SHOULDER_MIN_DIST && rightShoulderDist >= RELOCATE_SHOULDER_MIN_DIST &&
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
          handDist <= RELOCATE_HAND_MAX_DIST &&
          leftHand.Position.Y > spine.Position.Y && rightHand.Position.Y > spine.Position.Y &&
          leftHand.Position.Y < leftShoulder.Position.Y && rightHand.Position.Y < rightShoulder.Position.Y &&
          leftShoulderDist >= RELOCATE_SHOULDER_MIN_DIST && rightShoulderDist >= RELOCATE_SHOULDER_MIN_DIST &&
          (possibleRelocateGestureLeftward && rightHand.Position.X > rightShoulder.Position.X) ||
          (possibleRelocateGestureRightward && leftHand.Position.X < leftShoulder.Position.X)) {
        definitelyInRelocateGesture = true;
        possibleRelocateGestureLeftward = possibleRelocateGestureRightward = false;
        mhbCore.DebugGesture("Relocate");
      }

      if ((possibleRelocateGestureLeftward || possibleRelocateGestureRightward) &&
          (DateTime.Now - startRelocateTime).TotalMilliseconds >= RELOCATE_INTERVAL) {
        possibleRelocateGestureLeftward = possibleRelocateGestureRightward = false;
      }

      return definitelyInRelocateGesture;
    }

    private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
    {
      mhbCore.DebugThread("kinect skeleton frame");

      Skeleton[] skeletons = new Skeleton[0];

      using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame()) {
        if (skeletonFrame != null) {
          skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
          skeletonFrame.CopySkeletonDataTo(skeletons);
        }
      }

      bool isTracking = false;
      foreach (Skeleton data in skeletons) {
        if (SkeletonTrackingState.Tracked == data.TrackingState &&
            (lastFoundUserId == data.TrackingId || (DateTime.Now - lastFoundUserTime).TotalSeconds >= FIND_USER_INTERVAL)) {
          if (lastFoundUserId != data.TrackingId) {
            lock (mhbState.Lock) {
              mhbState.g.playFoundSound = true;
            }
            lastFoundUserId = data.TrackingId;
            lastFoundUserTime = DateTime.Now;
            playedSoundSinceLastUserLost = false;
          }

          isTracking = true;

          Joint trackingJoint = FindJoint(data.Joints, JointType.Spine);
          Point3D trackingPoint =
            new Point3D(trackingJoint.Position.X, trackingJoint.Position.Y, trackingJoint.Position.Z);

          lock (mhbState.Lock) {
            mhbState.g.userPosition = trackingPoint;
            mhbState.g.isInStopGesture |= IsInStopGesture(data);
            mhbState.g.isInGoGesture |= IsInGoGesture(data);
            mhbState.g.isInSaveGesture |= IsInSaveGesture(data);
            mhbState.g.isInRelocateGesture |= IsInRelocateGesture(data);
          }

          break;
        }
      }

      lock (mhbState.Lock) {
        mhbState.g.isTracking = isTracking;
        if (!isTracking && !playedSoundSinceLastUserLost) {
          playedSoundSinceLastUserLost = true;
          mhbState.g.playLostSound = true;
        }
      }
    }
    #endregion Skeletal processing

    #region constants
    private enum Axis {
      None = 0,
      X = 1,
      Y = 2,
      Z = 4,
    };

    private const int FIND_USER_INTERVAL = 2;

    private const int STOP_SUCCESSIVE_GESTURES = 3;

    private const double STOP_ELBOW_ANGLE_TOL = 10.0;
    private const double STOP_HAND_MIN_DIST = 0.5;

    private const double GO_HAND_MIN_DIFF = 0.25;
    private const double GO_HAND_MAX_DIFF = 0.5;
    private const double GO_HAND_START_MIN_DIST = 0.35;
    private const double GO_HAND_START_MAX_DIST = 0.7;
    private const double GO_HAND_END_MIN_DIST = 0.1;
    private const double GO_HAND_END_MAX_DIST = 0.3;
    private const int GO_INTERVAL = 500;

    private const double SAVE_HAND_START_DIST = 0.5;
    private const double SAVE_HAND_END_DIST = 0.1;
    private const int SAVE_INTERVAL = 500;

    private const double RELOCATE_HAND_MAX_DIST = 0.4;
    private const double RELOCATE_SHOULDER_MIN_DIST = 0.05;
    private const int RELOCATE_INTERVAL = 750;
    #endregion constants

    #region Private state
    private KinectSensor mKinect = null;
    private SpeechRecognitionEngine mSpeechRecognizer;
    private DateTime lastFoundUserTime;
    private bool playedSoundSinceLastUserLost = true;
    private int lastFoundUserId;
    private WebClient webClient = new WebClient();

    private int numSuccessiveStopGestures = 0;

    private bool possibleGoGesture = false;
    private DateTime startGoTime;

    private bool possibleSaveGesture = false;
    private DateTime startSaveTime;

    private bool possibleRelocateGestureLeftward = false;
    private bool possibleRelocateGestureRightward = false;
    private DateTime startRelocateTime;
    #endregion Private state
  }
}
