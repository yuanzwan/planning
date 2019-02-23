/* Copyright (C) 2018 RobotWits, LLC - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Written by Jonathan Butzke <instructor@robotwits.com>, January, 2018
 */

#include "an_map_manager/map_manager.h"


geometry_msgs::Point rotZ(double angle) {
  geometry_msgs::Point pt2;

  pt2.x = cos(angle);
  pt2.y = sin(angle);

  return pt2;
}


an_messages::lanes ReadMapFile(void) {
  an_messages::lanes l_;
  int numlanes = 0;

  std::string fname;
  while (!ros::param::has("MAP_FILE")) {
    ROS_WARN("[map] sleeping while waiting");
    ros::Duration(0.1).sleep();
  }

  ros::param::get("/MAP_FILE", fname);
  ROS_DEBUG("[map] %s", fname.c_str() );

  FILE* fp = fopen(fname.c_str(), "r");
  if (fp == NULL) {
    ROS_WARN("[map] No map file!  Exiting!!");
    l_.lanes.clear();
    fclose(fp);
    return l_;
  }

  int sz = fscanf(fp, "%i", &numlanes);
  if (sz == 0) {
    ROS_WARN("[map] Error with map file (1)!");
    l_.lanes.clear();
    fclose(fp);
    return l_;
  }

  l_.lanes.resize(numlanes);

  geometry_msgs::Point pt;

  for (int idx =-0; idx < numlanes; idx++) {
    int numpoints=0;
    sz = fscanf(fp, "%i %lf %i %i %i %i", &l_.lanes[idx].id, &l_.lanes[idx].width, &l_.lanes[idx].rightneighbor, &l_.lanes[idx].leftneighbor, &l_.lanes[idx].nextlane, &numpoints);
    if (sz != 6) {
      ROS_WARN("[map] Error with map file (2)!");
      l_.lanes.clear();
      fclose(fp);
      return l_;
    }
    l_.lanes[idx].centerline.resize(numpoints);
    int sidx;
    for (sidx=0; sidx < numpoints; sidx++) {
      sz = fscanf(fp, "%lf %lf %lf", &l_.lanes[idx].centerline[sidx].x, &l_.lanes[idx].centerline[sidx].y, &l_.lanes[idx].centerline[sidx].z);
      ROS_INFO("[map] cl: %f %f",  l_.lanes[idx].centerline[sidx].x, l_.lanes[idx].centerline[sidx].y);
      if (sz != 3) {
        ROS_WARN("[map] Error with map file (3)!");
        l_.lanes.clear();
        fclose(fp);
        return l_;
      }
      if (sidx > 0) {
        double angle = atan2(l_.lanes[idx].centerline[sidx].y-l_.lanes[idx].centerline[sidx-1].y, l_.lanes[idx].centerline[sidx].x-l_.lanes[idx].centerline[sidx-1].x);
        pt = rotZ(angle + M_PI/2);
        pt.x = pt.x*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].x;
        pt.y = pt.y*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].y;
        l_.lanes[idx].rightedge.push_back(pt);
        ROS_DEBUG("[map] right edge: %f %f",  l_.lanes[idx].rightedge.back().x, l_.lanes[idx].rightedge.back().y);
        pt = rotZ(angle - M_PI/2);
        pt.x = pt.x*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].x;
        pt.y = pt.y*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].y;
        l_.lanes[idx].leftedge.push_back(pt);
        ROS_DEBUG("[map] left edge: %f %f",  l_.lanes[idx].leftedge.back().x, l_.lanes[idx].leftedge.back().y);
      }
    }
    double angle = atan2(l_.lanes[idx].centerline[sidx-1].y-l_.lanes[idx].centerline[sidx-2].y, l_.lanes[idx].centerline[sidx-1].x-l_.lanes[idx].centerline[sidx-2].x);
    pt = rotZ(angle + M_PI/2);
    pt.x = pt.x*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].x;
    pt.y = pt.y*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].y;
    l_.lanes[idx].rightedge.push_back(pt);
    ROS_DEBUG("[map] right edge: %f %f",  l_.lanes[idx].rightedge.back().x, l_.lanes[idx].rightedge.back().y);
    pt = rotZ(angle - M_PI/2);
    pt.x = pt.x*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].x;
    pt.y = pt.y*l_.lanes[idx].width/2 + l_.lanes[idx].centerline[sidx-1].y;
    l_.lanes[idx].leftedge.push_back(pt);
    ROS_DEBUG("[map] left edge: %f %f",  l_.lanes[idx].leftedge.back().x, l_.lanes[idx].leftedge.back().y);
  }
  fclose(fp);
  return l_;
}

int main( int argc, char** argv )  {
  ros::init(argc, argv, "map_manager");
  ros::NodeHandle nh;
  ros::Rate r(0.5);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 5, true);
  ros::Publisher lanes_pub = nh.advertise<an_messages::lanes>("lanes", 1, true);
  an_messages::lanes L = ReadMapFile();
  ros::Duration(1).sleep();


  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.ns = "lanes";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.lifetime = ros::Duration();
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;


  while (ros::ok())
  {
    marker.header.stamp = ros::Time::now();
    for (int lidx = 0; lidx < L.lanes.size(); lidx++) {

      if (L.lanes[lidx].leftneighbor == -1) {
        marker.color.r = 0.0f;
        marker.id = L.lanes[lidx].id;
        marker.points = L.lanes[lidx].leftedge;
        marker_pub.publish(marker);
        ros::Duration(0.1).sleep();
      }

      if (L.lanes[lidx].rightneighbor == -1) {
        marker.color.r = 0.0f;
      } else {
        marker.color.r = 1.0f;
      }
      marker.id = L.lanes[lidx].id + L.lanes.size() + 1;
      marker.points = L.lanes[lidx].rightedge;
      marker_pub.publish(marker);
      ros::Duration(0.1).sleep();
    }
    lanes_pub.publish(L);
    r.sleep();
  }
}
