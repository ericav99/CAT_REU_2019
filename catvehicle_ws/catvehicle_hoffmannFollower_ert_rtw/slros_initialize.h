#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block catvehicle_hoffmannFollower/Subscribe3
extern SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_catvehicle_hoffmannFollower_nav_msgs_Odometry> Sub_catvehicle_hoffmannFollower_13;

// For Block catvehicle_hoffmannFollower/Publish
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_catvehicle_hoffmannFollower_geometry_msgs_Twist> Pub_catvehicle_hoffmannFollower_9;

void slros_node_init(int argc, char** argv);

#endif
