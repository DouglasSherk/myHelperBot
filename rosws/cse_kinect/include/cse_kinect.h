#ifndef CSEKINECT_H
#define CSEKINECT_H

// System libraries.
#include <cstdlib>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Local includes.
#include "cse_kinect/PoseData.h"

class CseKinect
{
public:
  CseKinect();
  ~CseKinect();

  void publishPoseData(ros::Publisher *pubPoseData);

  void lookForPoses();

private:
  //! A listener for the transform topic.
  tf::TransformListener listener;

  //! Transforms for each body part.
  tf::StampedTransform tf_right_shoulder;
  tf::StampedTransform tf_right_elbow;
  tf::StampedTransform tf_right_hand;
  tf::StampedTransform tf_left_shoulder;
  tf::StampedTransform tf_left_elbow;
  tf::StampedTransform tf_left_hand;
  tf::StampedTransform tf_neck;
  tf::StampedTransform tf_right_hip;
  tf::StampedTransform tf_left_hip;
  tf::StampedTransform tf_right_foot;
  tf::StampedTransform tf_left_foot;
  tf::StampedTransform tf_right_knee;
  tf::StampedTransform tf_left_knee;
  tf::StampedTransform tf_head;
  tf::StampedTransform tf_torso;

  bool mStop;
  bool mGo;
  bool mTooFar;
  bool mTooClose;
  bool mTooClockwise;
  bool mTooCClockwise;

  double x_right_shoulder;
  double y_right_shoulder;
  double x_right_elbow;
  double y_right_elbow;
  double x_right_hand;
  double y_right_hand;
  double x_left_shoulder;
  double y_left_shoulder;
  double x_left_elbow;
  double y_left_elbow;
  double x_left_hand;
  double y_left_hand;
  double x_neck;
  double y_neck;
  double x_right_hip;
  double y_right_hip;
  double x_left_hip;
  double y_left_hip;
  double x_right_foot;
  double y_right_foot;
  double x_left_foot;
  double y_left_foot;
  double x_right_knee;
  double y_right_knee;
  double x_left_knee;
  double y_left_knee;
  double x_head;
  double y_head;
  double x_torso;
  double y_torso;
};

#endif // CSEKINECT_H
