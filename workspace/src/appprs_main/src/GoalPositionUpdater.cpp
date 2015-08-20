/*
 * GoalPositionUpdater.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: vgrabe
 */

#include <Eigen/Dense>
#include "appprs_main/GoalPositionUpdater.h"

GoalPositionUpdater::GoalPositionUpdater() {
  goal_publisher_  = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  goal_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GoalPositionUpdater::goal_callback, this);
  timer_ = nh_.createTimer(ros::Duration(0.5), &GoalPositionUpdater::timer_callback, this);
  distance_threshold_ = 1.0;
}

GoalPositionUpdater::~GoalPositionUpdater() {
  // TODO Auto-generated destructor stub
}

void GoalPositionUpdater::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  last_goal_ = current_goal_;
  current_goal_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void GoalPositionUpdater::timer_callback(const ros::TimerEvent& e) {
  tf::StampedTransform transform;
  try{
    tf_listener_.lookupTransform("/base_link", "/map", ros::Time(0), transform);
  }
  catch (tf::TransformException& ex){
    return;
    //ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }

  Eigen::Vector3d robot_pose = Eigen::Vector3d (transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
  double distance = (current_goal_-robot_pose).norm();
  if(distance<distance_threshold_){
    std::cout << "Robot is within "<< distance_threshold_ << "m of goal" << std::endl;
  }
}
