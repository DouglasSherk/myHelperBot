#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <sstream>

#include "cse_gestures.h"

int gMotorSpeed = 200;


/*------------------------------------------------------------------------------
 * CseGestures()
 * Constructor.
 *----------------------------------------------------------------------------*/

CseGestures::CseGestures()
{
} // end CseGestures()


/*------------------------------------------------------------------------------
 * ~CseGestures()
 * Destructor.
 *----------------------------------------------------------------------------*/

CseGestures::~CseGestures()
{
} // end ~CseGestures()


/*---------------------------------------------------------------------
* poseDataCallback()
* Callback for when a pose is found.
* -------------------------------------------------------------------*/

void CseGestures::poseDataCallback(const cse_kinect::PoseData::ConstPtr &msg)
{
  mStop = msg->mStop;
  mGo   = msg->mGo;
} // end poseDataCallback()


/*---------------------------------------------------------------------
* ultrasonicCallback()
* Callback for when the Arduino sends ultrasonic data.
* -------------------------------------------------------------------*/

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


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cse_gestures");
  ros::NodeHandle n;

  // Declare variables.
  CseGestures *cse_gestures;

  // Set up a CseGestures object.
  cse_gestures = new CseGestures();

  ros::Publisher pubMotorA = n.advertise<std_msgs::Int32>("motorA", 1000);
  ros::Publisher pubMotorB = n.advertise<std_msgs::Int32>("motorB", 1000);

  ros::Subscriber subUltrasonic = n.subscribe("ultrasonic", 1000, ultrasonicCallback);
  ros::Subscriber subPoseData = n.subscribe("poseData", 1000, &CseGestures::poseDataCallback, cse_gestures);

  // Tell ROS to run this node at the desired rate.
  ros::Rate r(20);

  while (n.ok())
  {
    std_msgs::Int32 motorA, motorB;
    motorA.data = gMotorSpeed;
    motorB.data = gMotorSpeed;

    if (cse_gestures->mStop)
    {
      ros::Time stopTime = ros::Time::now();
    }
    if (cse_gestures->mGo)
    {
      ros::Time goTime = ros::Time::now();
    }

    pubMotorA.publish(motorA);
    pubMotorB.publish(motorB);

    // Let ROS run all of its background threads to send out data now.
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
