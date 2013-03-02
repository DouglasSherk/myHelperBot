#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

const float SPEED_NONE = 0.0;
const float SPEED_MAX = 100.0;

const float DISTANCE_MIN = 1.4;
const float DISTANCE_MAX = 1.6;

const float ROTATION_MIN = -0.2;
const float ROTATION_MAX = 0.2;

/** Map a rotation of 0.2 to max speed. */
const float ROTATION_FACTOR = SPEED_MAX / 5.0;
/** Map a distance of 0.8 to max speed. */
const float DISTANCE_FACTOR = SPEED_MAX / 5.0;

float gLastDistance = 0.0;
float gLastRotation = 0.0;

float gDistance = 0.0;
float gRotation = 0.0;

bool gStopped = false;

int gConsecutiveStop = 0;
int gConsecutiveGo = 0;

int gConsecutiveSame = 0;

void kinectCallback(const std_msgs::String::ConstPtr& msg)
{
  // x: rotation
  // y: elevation
  // z: depth
  int stop, go, save;
  float distances[3];
  sscanf(msg->data.c_str(), "%i, %i, %i, %f, %f, %f",
         &stop, &go, &save, &distances[0], &distances[1], &distances[2]);

  if (!gStopped && !!stop && ++gConsecutiveStop >= 5) { gStopped = true; gConsecutiveStop = 0; }
  if (gStopped && !!go && ++gConsecutiveGo >= 5) { gStopped = false; gConsecutiveGo = 0; }

  ROS_WARN(msg->data.c_str());
  char buf[256];
  sprintf(buf, "%d", gStopped);
  ROS_WARN(buf);

  gLastRotation = gRotation;
  gLastDistance = gDistance;

  gRotation = distances[0];
  gDistance = distances[2];

  if (fabsf(gRotation - gLastRotation) < 0.00001 &&
      fabsf(gDistance - gLastDistance) < 0.00001) {
    gConsecutiveSame++;
  } else {
    gConsecutiveSame = 0;
  }
}

void ultrasonicCallback(const std_msgs::String::ConstPtr& msg)
{
  int distances[6];
  sscanf(msg->data.c_str(), "%d, %d, %d, %d, %d, %d", 
         &distances[0], &distances[1], &distances[2],
         &distances[3], &distances[4], &distances[5]);

  for (int i = 0; i < 6; i++) {
    if (distances[i] > 0 && distances[i] < 30) {
      // XXX: set speed?
      break;
    }
  }
}

void calculateMotorSpeeds(int& motorA, int& motorB)
{
  motorA = SPEED_NONE;
  motorB = SPEED_NONE;

  if ((gConsecutiveSame >= 3) ||
      (gRotation < 0.01 && gDistance < 0.01) ||
      gStopped) {
    return;
  }

  float rotationGettingCloser = (float) (fabsf(gRotation) < fabsf(gLastRotation));
  float distanceGettingCloser = (float) (fabsf(gDistance) < fabsf(gLastDistance));

  if (gRotation > ROTATION_MAX) {
    float propSpeed = 70.0 + (gRotation - ROTATION_MAX) * ROTATION_FACTOR - rotationGettingCloser * 30.0;
    motorA = (int) propSpeed;
    motorB = (int) -propSpeed;
  } else if (gRotation < ROTATION_MIN) {
    float propSpeed = 70.0 + (ROTATION_MIN - gRotation) * ROTATION_FACTOR - rotationGettingCloser * 30.0;
    motorA = (int) -propSpeed;
    motorB = (int) propSpeed;
  } else if (gDistance > DISTANCE_MAX) {
    float propSpeed = 70.0 + (gDistance - DISTANCE_MAX) * DISTANCE_FACTOR - distanceGettingCloser * 30.0;
    motorA = (int) propSpeed * 0.9;
    motorB = (int) propSpeed;
  } else if (gDistance < DISTANCE_MIN) {
    float propSpeed = 70.0 + (DISTANCE_MIN - gDistance) * DISTANCE_FACTOR - distanceGettingCloser * 30.0;
    motorA = (int) -propSpeed * 0.9;
    motorB = (int) -propSpeed;
  } 

  if (motorA < -SPEED_MAX) motorA = -SPEED_MAX;
  else if (motorA > SPEED_MAX) motorA = SPEED_MAX;
  if (motorB < -SPEED_MAX) motorB = -SPEED_MAX;
  else if (motorB > SPEED_MAX) motorB = SPEED_MAX;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "myhelperbot_core");

  ros::NodeHandle n;

  ros::Publisher pubMotorA = n.advertise<std_msgs::Int32>("motorA", 10);
  ros::Publisher pubMotorB = n.advertise<std_msgs::Int32>("motorB", 10);

  ros::Subscriber subUltrasonic = n.subscribe("ultrasonic", 10, ultrasonicCallback);
  ros::Subscriber subKinect = n.subscribe("kinect", 10, kinectCallback);

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    std_msgs::Int32 motorA, motorB;

    calculateMotorSpeeds(motorA.data, motorB.data);
    motorA.data *= 2.55;
    motorB.data *= 2.55;

    pubMotorA.publish(motorA);
    pubMotorB.publish(motorB);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
