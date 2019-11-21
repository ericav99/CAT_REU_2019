#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "catvehicle_hoffmannFollower";

// For Block catvehicle_hoffmannFollower/Subscribe3
SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_catvehicle_hoffmannFollower_nav_msgs_Odometry> Sub_catvehicle_hoffmannFollower_13;

// For Block catvehicle_hoffmannFollower/Publish
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_catvehicle_hoffmannFollower_geometry_msgs_Twist> Pub_catvehicle_hoffmannFollower_9;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

