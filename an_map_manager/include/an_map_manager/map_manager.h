/* Copyright (C) 2018 RobotWits, LLC - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Written by Jonathan Butzke <instructor@robotwits.com>, January, 2018
 */

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "an_messages/lanes.h"
#include "an_messages/lane.h"
#include <vector>
#include <string>


// class Lane_c {
// public:
//   int id;
//   double width;
//   int rightneighbor;
//   int leftneighbor;
//   int nextlane;
//   std::vector<geometry_msgs::Point> centerline;
//   std::vector<geometry_msgs::Point> leftedge;
//   std::vector<geometry_msgs::Point> rightedge;
//   };
