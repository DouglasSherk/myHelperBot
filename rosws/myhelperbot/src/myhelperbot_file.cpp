#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <stdlib.h>

#include <sstream>

char buffer[512];

void readFile()
{
  int fileSize = 0;
  FILE *fh = fopen("/tmp/ros.txt", "rb");
  if (fh != NULL) {
    fseek(fh, 0L, SEEK_END);
    fileSize = ftell(fh);
    fseek(fh, 0L, SEEK_SET);

    size_t size = fread(buffer, fileSize, 1, fh);
    buffer[fileSize] = 0;

    fclose(fh);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher pubKinect = n.advertise<std_msgs::String>("kinect", 10);

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    std_msgs::String kinect;

    readFile();
    kinect.data = buffer;

    pubKinect.publish(kinect);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
