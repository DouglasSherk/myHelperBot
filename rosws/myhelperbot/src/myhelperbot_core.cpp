#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

const float SPEED_NONE = 100.0;
const float SPEED_MAX = 2500.0;

const float DISTANCE_MIN = 1.4;
const float DISTANCE_MAX = 1.6;

const float ROTATION_MIN = -0.2;
const float ROTATION_MAX = 0.2;

/** Map a rotation of 5.0 to max speed. */
const float ROTATION_FACTOR = SPEED_MAX / 1.0;
/** Map a distance of 5.0 to max speed. */
const float DISTANCE_FACTOR = SPEED_MAX / 1.0;

float gLastDistance = 0.0;
float gLastRotation = 0.0;

float gDistance = 0.0;
float gRotation = 0.0;

bool gStopped = false;

int gConsecutiveStop = 0;

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
  if (gStopped && !!go) { gStopped = false; }

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

void calculateMotorSpeeds(int& motorR, int& motorL)
{
  // XXX: Remove these when we have motor speed control working.
  const float MIN_SPEED = 1000.0, CLOSE_SPEED = 500.0;

  static int prevSpeed[2];

  motorR = SPEED_NONE;
  motorL = SPEED_NONE;

  if ((gConsecutiveSame >= 4) ||
      (fabsf(gRotation) < 0.01 && fabsf(gDistance) < 0.01) ||
      gStopped) {
    return;
  }

  float rotationGettingCloser = (float) (fabsf(gRotation) < fabsf(gLastRotation));
  float distanceGettingCloser = (float) (fabsf(gDistance) < fabsf(gLastDistance));

  if (gRotation > ROTATION_MAX) {
    float propSpeed = MIN_SPEED + (gRotation - ROTATION_MAX) * ROTATION_FACTOR - rotationGettingCloser * CLOSE_SPEED;
    motorR = (int) propSpeed;
    motorL = (int) -propSpeed;
  } else if (gRotation < ROTATION_MIN) {
    float propSpeed = MIN_SPEED + (ROTATION_MIN - gRotation) * ROTATION_FACTOR - rotationGettingCloser * CLOSE_SPEED;
    motorR = (int) -propSpeed;
    motorL = (int) propSpeed;
  } else if (gDistance > DISTANCE_MAX) {
    float propSpeed = MIN_SPEED + (gDistance - DISTANCE_MAX) * DISTANCE_FACTOR - distanceGettingCloser * CLOSE_SPEED;
    motorR = (int) propSpeed * 0.9;
    motorL = (int) propSpeed;
  } else if (gDistance < DISTANCE_MIN) {
    float propSpeed = MIN_SPEED + (DISTANCE_MIN - gDistance) * DISTANCE_FACTOR - distanceGettingCloser * CLOSE_SPEED;
    motorR = (int) -propSpeed * 0.9;
    motorL = (int) -propSpeed;
  } 

  // Max out throttle on start from stop.
  // XXX: Remove these when we have motor speed control working.
  if (prevSpeed[0] == 0 && prevSpeed[1] == 0 && motorR != 0 && motorL != 0) { motorR *= 999; motorL *= 999; }

  if (motorR < -SPEED_MAX) motorR = -SPEED_MAX;
  else if (motorR > SPEED_MAX) motorR = SPEED_MAX;
  if (motorL < -SPEED_MAX) motorL = -SPEED_MAX;
  else if (motorL > SPEED_MAX) motorL = SPEED_MAX;

  prevSpeed[0] = motorR;
  prevSpeed[1] = motorL;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "myhelperbot_core");

  ros::NodeHandle n;

  ros::Publisher pubMotorR = n.advertise<std_msgs::Int32>("motorR", 10);
  ros::Publisher pubMotorL = n.advertise<std_msgs::Int32>("motorL", 10);

  ros::Subscriber subUltrasonic = n.subscribe("ultrasonic", 10, ultrasonicCallback);
  ros::Subscriber subKinect = n.subscribe("kinect", 10, kinectCallback);

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    std_msgs::Int32 motorR, motorL;

    calculateMotorSpeeds(motorR.data, motorL.data);

    char buf[256];
    sprintf(buf, "Speeds: %d %d", motorR.data, motorL.data);
    ROS_WARN(buf);

    pubMotorR.publish(motorR);
    pubMotorL.publish(motorL);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
