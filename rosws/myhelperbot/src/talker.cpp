#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

int gMotorSpeed = 200;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher pubMotorA = n.advertise<std_msgs::Int32>("motorA", 1000);
  ros::Publisher pubMotorB = n.advertise<std_msgs::Int32>("motorB", 1000);

  ros::Subscriber subUltrasonic = n.subscribe("ultrasonic", 1000, ultrasonicCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
/**
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
*/

    std_msgs::Int32 motorA, motorB;
    motorA.data = gMotorSpeed;
    motorB.data = gMotorSpeed;
    
    ROS_INFO("motors: %d %d", motorA.data, motorB.data);

    pubMotorA.publish(motorA);
    pubMotorB.publish(motorB);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
