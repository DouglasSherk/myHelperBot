#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

enum {
  Speed_None = 0,
  Speed_Low = 140,
  Speed_High = 200,
  Speed_Max = 255,
};

bool gTooClose = false;
bool gTooFar = false;
bool gTooClw = false;
bool gTooCClw = false;

int gMotorSpeed = 200;

void kinectCallback(const std_msgs::String::ConstPtr& msg)
{
  // x: rotation
  // y: elevation
  // z: depth
  static float lastDistances[3];
  float distances[3];
  sscanf(msg->data.c_str(), "%f, %f, %f",
         &distances[0], &distances[1], &distances[2]);

  if (fabsf(distances[0] - lastDistances[0]) < 0.01 &&
      fabsf(distances[1] - lastDistances[1]) < 0.01 &&
      fabsf(distances[2] - lastDistances[2]) < 0.01) {
    //return;
  }

  ROS_WARN(msg->data.c_str());

  if (distances[0] < -0.20) {
    gTooClw = true;
  } else if (distances[0] > 0.20) {
    gTooCClw = true;
  }

  if (distances[2] < 1.3) {
    gTooClose = true;
  } else if (distances[2] > 1.55) {
    gTooFar = true;
  }

  lastDistances[0] = distances[0];
  lastDistances[1] = distances[1];
  lastDistances[2] = distances[2];
}

void ultrasonicCallback(const std_msgs::String::ConstPtr& msg)
{
  int distances[6];
  sscanf(msg->data.c_str(), "%d, %d, %d, %d, %d, %d", 
         &distances[0], &distances[1], &distances[2],
         &distances[3], &distances[4], &distances[5]);

  gMotorSpeed = 200;
  for (int i = 0; i < 6; i++) {
    if (distances[i] > 0 && distances[i] < 30) {
      gMotorSpeed = 0;
      break;
    }
  }
}

void calculateMotorSpeeds(int& motorA, int& motorB)
{
  motorA = Speed_None;
  motorB = Speed_None;

  ROS_WARN("%d %d %d %d", gTooFar, gTooClose, gTooClw, gTooCClw);

  if (gTooClw) {
    motorA = -Speed_High;
    motorB = Speed_High;
  } else if (gTooCClw) {
    motorA = Speed_High;
    motorB = -Speed_High;
  } else if (gTooFar) {
    motorA = Speed_High;
    motorB = Speed_High;
  } else if (gTooClose) {
    motorA = -Speed_High;
    motorB = -Speed_High;
  } 
}

void resetFlags()
{
  gTooFar = false;
  gTooClose = false;
  gTooClw = false;
  gTooCClw = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher pubMotorA = n.advertise<std_msgs::Int32>("motorA", 10);
  ros::Publisher pubMotorB = n.advertise<std_msgs::Int32>("motorB", 10);

  ros::Subscriber subUltrasonic = n.subscribe("ultrasonic", 10, ultrasonicCallback);
  ros::Subscriber subKinect = n.subscribe("kinect", 10, kinectCallback);

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    std_msgs::Int32 motorA, motorB;

    calculateMotorSpeeds(motorA.data, motorB.data);
    resetFlags();

    pubMotorA.publish(motorA);
    pubMotorB.publish(motorB);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
