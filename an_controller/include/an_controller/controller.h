/* Copyright (C) 2018 RobotWits, LLC - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Written by Jonathan Butzke <instructor@robotwits.com>, January, 2018
 */

#include "ros/ros.h"
#include <vector>
#include <cstdio>
#include "an_messages/trajectory.h"
#include "an_messages/traj_pt.h"
#include "an_messages/observation.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

class Controller_c {
public:
  bool new_traj_ = true;
  std::vector<an_messages::traj_pt> traj_;
  nav_msgs::Path path_;
  int current_idx_;
  geometry_msgs::PoseStamped pose_;
  ros::Time prevTime_;
  visualization_msgs::Marker marker_;
  int loopcount_;
  float current_belief_, actual_intent_;
  an_messages::observation observation_;
  bool USEACTION_;


  ros::NodeHandle nh_;
  ros::Subscriber trajectory_sub_, action_sub_;
  ros::Publisher  poseStamped_pub_, path_pub_, marker_pub_, observation_pub_;
  tf::TransformBroadcaster br;
  tf::Transform transform_;

  bool Init_(void);
  void Loop_(void);
  void trajectory_callback_(const an_messages::trajectory::ConstPtr& msg);
  void action_callback_(const an_messages::trajectory::ConstPtr& msg);
  void SendPath(void);
};
