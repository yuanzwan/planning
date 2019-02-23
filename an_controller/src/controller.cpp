/* Copyright (C) 2018 RobotWits, LLC - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Written by Jonathan Butzke <instructor@robotwits.com>, January, 2018
 */
#include "an_controller/controller.h"

void Controller_c::trajectory_callback_(const an_messages::trajectory::ConstPtr& msg) {
  ROS_DEBUG("[controller] Entering trajectory callback");

  for (int i=0; i < msg->traj.size(); i++) {
    traj_.push_back(msg->traj[i]);
  }
  new_traj_ = true;
  ROS_WARN("[controller] Received new trajectory, length: %li", traj_.size() );
}

void Controller_c::action_callback_(const an_messages::trajectory::ConstPtr& msg) {
  ROS_DEBUG("[controller] Entering action callback");
  USEACTION_=true;

  //traj_.clear();   really should clear the old trajectory

  for (int i=0; i < msg->traj.size(); i++) {
    traj_.push_back(msg->traj[i]);
  }

  if (msg->traj[0].velocity.linear.x < 0) {
    // flash headlights
    current_belief_ = actual_intent_;  //TODO: check distance
//     observation_.x = traj_[0].position.x;
//     observation_.y = traj_[0].position.y;
//     observation_.Change = actual_intent_;
//     observation_pub_.publish(observation_);
  }

  new_traj_ = true;
  ROS_WARN("[controller] Received new action, length: %li", traj_.size() );
}

void Controller_c::SendPath(void) {
  path_.poses.resize(traj_.size());
  path_.header = path_.poses[0].header;
  for (int pidx=0; pidx < path_.poses.size(); pidx++) {
    path_.poses[pidx].header = traj_[pidx].header;

    path_.poses[pidx].pose.position.x = traj_[pidx].position.x;
    path_.poses[pidx].pose.position.y = traj_[pidx].position.y;
    path_.poses[pidx].pose.position.z = 0;

    path_.poses[pidx].pose.orientation = tf::createQuaternionMsgFromYaw(traj_[pidx].position.theta);
  }
}



bool Controller_c::Init_(void) {
  ROS_DEBUG("[controller] Entering init");
  USEACTION_=false;
  poseStamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("vehicle_pose", 1000);
  path_pub_ = nh_.advertise<nav_msgs::Path>("vehicle_path", 1000);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 5, false);
  observation_pub_ = nh_.advertise<an_messages::observation>("observation", 2, true);

  trajectory_sub_ = nh_.subscribe("planner_trajectory", 1, &Controller_c::trajectory_callback_, this);

  action_sub_ = nh_.subscribe("policy_step", 1, &Controller_c::action_callback_, this);


  current_idx_ = 0;
  new_traj_ = false;
  pose_.header.frame_id = "/map";
  pose_.pose.position.x = 0;
  pose_.pose.position.y = 0;
  pose_.pose.position.z = 0;
  pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  transform_.setOrigin(tf::Vector3(0,0,0));
  transform_.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "ego"));

  marker_.header.frame_id = "map";//"/ego";//"/map";
  marker_.header.seq = 0;
  //marker_.type = visualization_msgs::Marker::CUBE;
  marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_.mesh_use_embedded_materials = true;
  //marker_.mesh_resource = "package://an_planner/meshes/Generic_Coupe_v02.dae";
  marker_.mesh_resource = "package://an_scenario/meshes/Generic_Coupe_v02.dae";
  //marker_.mesh_resource = "package://an_controller/config/egocar.dae";

  marker_.ns = "ego";
  marker_.id = 1;
  marker_.scale.x = 0.0010;//4.8;  //TODO: make params
  marker_.scale.y = 0.0010;//1.8;
  marker_.scale.z = 0.0010;//1.4;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.lifetime = ros::Duration();
  marker_.color.r = 0;//0.0f;  for meshes
  marker_.color.g = 0;//1.0f;
  marker_.color.b = 0;//0.0f;
  marker_.color.a = 0;//1.0f;

  prevTime_ = ros::Time::now();
  loopcount_ = 0;
  current_belief_=an_messages::observation::UNSURE;

  observation_.header.stamp = ros::Time::now();
  observation_.x = marker_.pose.position.x;
  observation_.y = marker_.pose.position.y;
  observation_.Change = an_messages::observation::UNSURE;
  observation_pub_.publish(observation_);


  if (!nh_.hasParam("/driver_intends_to_change_lanes")) {
    ros::Duration(5).sleep();
  }

  nh_.param("/driver_intends_to_change_lanes", actual_intent_, an_messages::observation::UNSURE);
  ROS_ERROR("intent: %f", actual_intent_);


  ROS_DEBUG("[controller] Exiting init");
  return true;
}


void Controller_c::Loop_(void) {
  ROS_DEBUG("[controller] Entering loop");
  ros::Rate loop_rate(150);
  while(ros::ok() ) {
    ROS_DEBUG("[controller] Entering while");
    if (new_traj_) {
      SendPath();
      current_idx_ = 0;
      pose_.pose.position.x = traj_[0].position.x;
      pose_.pose.position.y = traj_[0].position.y;
      pose_.pose.orientation = tf::createQuaternionMsgFromYaw(traj_[0].position.theta);

      path_.header.stamp = ros::Time::now();
      path_.header.frame_id = "/map";
      path_pub_.publish(path_);

      new_traj_ = false;
      prevTime_ = ros::Time::now();
    }

    loopcount_++;
    if (loopcount_ > 20) {
      loopcount_ = 0;
      path_.header.stamp = ros::Time::now();
      path_.header.frame_id = "/map";
      path_pub_.publish(path_);
    }

    ROS_DEBUG("[controller] Entering traj if");
    if (traj_.size() != 0) {
      if (ros::Time::now() < traj_[0].header.stamp) {
        pose_.pose.position.x = traj_[0].position.x;
        pose_.pose.position.y = traj_[0].position.y;
        pose_.pose.orientation = tf::createQuaternionMsgFromYaw(traj_[0].position.theta);
      } else if (current_idx_  < traj_.size() ) {
        //TODO:  fix last point of trajectory
//         double velx = traj_[current_idx_].velocity.linear.x;
//         double vely = traj_[current_idx_].velocity.linear.y;
//
//         double vel = sqrt(velx*velx+vely*vely);
//         //TODO: limit velocity to car limits
//         ROS_DEBUG("[controller] vel: %f", vel);
//         ros::Duration delT(ros::Time::now() - prevTime_);
//         prevTime_ = ros::Time::now();
//         ROS_DEBUG("[controller] delT:%f", delT.toSec() );
//         double distToMove = delT.toSec()*vel;
//         //double distToMovex = delT.toSec()*velx;
//         //double distToMovey = delT.toSec()*vely;
//         double distToNextPoint = 0;
        int idx = current_idx_;
        if (idx < traj_.size() ) {
          ROS_DEBUG("[controller] current:%i idx:%i size%li", current_idx_, idx, traj_.size() );
          while ((idx < traj_.size() ) && (traj_[idx].header.stamp < ros::Time::now() )) {
            idx++;
          }
            pose_.pose.position.x = traj_[idx-1].position.x;
            pose_.pose.position.y = traj_[idx-1].position.y;
            pose_.pose.orientation = tf::createQuaternionMsgFromYaw(traj_[idx-1].position.theta);
            current_idx_ = idx;

          // removed physics
          //           distToNextPoint =sqrt((traj_[idx].position.x - pose_.pose.position.x)*(traj_[idx].position.x - pose_.pose.position.x) + (traj_[idx].position.y - pose_.pose.position.y)*(traj_[idx].position.y - pose_.pose.position.y));
          //
          //           ROS_DEBUG("[controller] initial next:%f total:%f", distToNextPoint, distToMove);
        }
        //removed physics
        //         while ((idx < traj_.size() ) && (distToNextPoint < distToMove)) {
        //           ROS_DEBUG("[controller] idx: %i next:%f total:%f", idx, distToNextPoint, distToMove);
        //           distToMove -= distToNextPoint;
        //           distToNextPoint = sqrt((traj_[idx].position.x - traj_[idx-1].position.x) * (traj_[idx].position.x - traj_[idx-1].position.x) + (traj_[idx].position.y - traj_[idx-1].position.y) * (traj_[idx].position.y - traj_[idx-1].position.y));
        //
        //           idx++;
        //         }

        //TODO:  may jump at last step
        ROS_DEBUG("[controller] finished while");
      //  ROS_DEBUG("[controller] idx: %i next:%f total:%f", idx, distToNextPoint, distToMove);

        if (idx == traj_.size() ) {
          ROS_DEBUG("[controller]  index reached the end of the trajectory");
          // at end stop there
          pose_.pose.position.x = traj_[idx-1].position.x;
          pose_.pose.position.y = traj_[idx-1].position.y;
          pose_.pose.orientation = tf::createQuaternionMsgFromYaw(traj_[idx-1].position.theta);
          current_idx_ = idx-1;

          observation_.header.stamp = traj_[idx-1].header.stamp;
          observation_.x = traj_[idx-1].position.x;
          observation_.y = traj_[idx-1].position.y;
          observation_.Change = current_belief_;
          observation_pub_.publish(observation_);

        } /*  //removed physics else {
        ROS_DEBUG("[controller] %i (%li)", idx, traj_.size() );
        /*
         *      distToMovey -= traj_[current_idx_+1].position.y - pose_.y;
         *
         *      double totaldist = sqrt(distToMovex*distToMovex+distToMovey*distToMovey);
         *      int i;
         *      ROS_WARN("[controller] Entering for");
         *
         *      for (i=current_idx_+2; i<traj_.size(); i++) {
         *        double distx = traj_[i].position.x - traj_[i-1].position.x;
         *        double disty = traj_[i].position.y - traj_[i-1].position.y;
         *
         *        double deldist = sqrt(distx*distx*disty*disty);
         *
         *        if (deldist < totaldist) {
         *          // interpolate
         *         double ratio = distToMove/distToNextPoint;
         *         double distx = traj_[idx].position.x - traj_[idx-1].position.x;
         *         double disty = traj_[idx].position.y - traj_[idx-1].position.y;
         *
         *         pose_.pose.position.x = traj_[idx-1].position.x + ratio*distx;
         *         pose_.pose.position.y = traj_[idx-1].position.y + ratio*disty;
         *         pose_.pose.orientation = tf::createQuaternionMsgFromYaw(traj_[idx-1].position.theta);
         *
         *         current_idx_ = idx;
         *
      }  */
        ROS_DEBUG("[controller] publishing");
        pose_.header.stamp = ros::Time::now();
        pose_.header.seq = current_idx_;
        poseStamped_pub_.publish(pose_);

        transform_.setOrigin(tf::Vector3(pose_.pose.position.x, pose_.pose.position.y, 0));
        transform_.setRotation(tf::Quaternion(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "ego"));

        marker_.header.stamp = ros::Time::now();
        marker_.pose = pose_.pose;
        //marker_.pose.position.z += marker_.scale.z/2;
 //marker_.pose.orientation.x =   0;
 //marker_.pose.orientation.y =   0;
 //marker_.pose.orientation.z =   0;
 //marker_.pose.orientation.w =   1;



        marker_pub_.publish(marker_);

       // ROS_WARN("ego: x:%2.1f y:%2.1f time: %3.3f (%3.3f)", marker_.pose.position.x, marker_.pose.position.y, marker_.header.stamp.toSec(), traj_[current_idx_-1].header.stamp.toSec() );
        ROS_DEBUG("[controller] finished for");
      }
    }

    //     transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
    //     transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ego"));

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");
  ros::start();
  ros::Duration(1).sleep();

  Controller_c controller;
  if (controller.Init_())
  {
    controller.Loop_();
  }
}

