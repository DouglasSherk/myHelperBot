#ifndef CSEGESTURES_H
#define CSEGESTURES_H

// System libraries.
#include <cstdlib>

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "cse_kinect/PoseData.h"

class CseGestures
{
public:
  CseGestures();
  ~CseGestures();

  //! Callback function to send out pose information.
  void poseDataCallback(const cse_kinect::PoseData::ConstPtr &msg);

  bool mStop;
  bool mGo;
};

#endif // CSEGESTURES_H
