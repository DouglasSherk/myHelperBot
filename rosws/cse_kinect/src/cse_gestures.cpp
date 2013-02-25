#include "cse_gestures.h"


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

  ros::Subscriber pose_data_sub = n.subscribe("poseData", 1000, &CseGestures::poseDataCallback, cse_gestures);

  // Tell ROS to run this node at the desired rate.
  ros::Rate r(20);

  while (n.ok())
  {
    if (cse_gestures->mStop)
    {
      ros::Time stopTime = ros::Time::now();
    }
    if (cse_gestures->mGo)
    {
      ros::Time goTime = ros::Time::now();
    }

    // Let ROS run all of its background threads to send out data now.
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
