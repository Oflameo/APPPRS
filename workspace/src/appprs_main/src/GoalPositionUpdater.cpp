/*
 * GoalPositionUpdater.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: vgrabe
 */

#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include "appprs_main/GoalPositionUpdater.h"
#include "appprs_main/localPP.h"
#include "appprs_main/p_control_path_planner.h"

GoalPositionUpdater::GoalPositionUpdater():
goals_received_(0), current_global_waypoint_index_(0), current_local_waypoint_index_(1){
  local_path_publisher_  = nh_.advertise<nav_msgs::Path>("/local_path", 10);
  global_path_publisher_ = nh_.advertise<nav_msgs::Path>("/global_path", 10);
  goal_publisher_  = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal2", 10);
  steer_command_publisher_ = nh_.advertise<std_msgs::Float32>("/steerCommand", 1000);
  speed_command_publisher_ = nh_.advertise<std_msgs::Float32>("/steerCommand", 1000);
  goal_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GoalPositionUpdater::goal_callback, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &GoalPositionUpdater::timer_callback, this);
  distance_threshold_ = 1.0;

  std::ifstream waypointFile ("/tmp/waypoints.txt");
  std::string line;
  if (waypointFile.is_open()) {
    while ( std::getline (waypointFile, line) ) {
      std::vector<float> number_list;
      boost::char_separator<char> sep(", ");
      boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
      for (const std::string& t : tokens) {
          number_list.push_back(std::atof(t.c_str()));
      }
      //create final datastructure
      if (number_list.size()<7) {
        std::cout << "Unable to parse way point list!" << std::endl;
        continue;
      }
      geometry_msgs::PoseStamped new_pose;
      new_pose.pose.position.x = number_list[0];
      new_pose.pose.position.y = number_list[1];
      new_pose.pose.position.z = number_list[2];
      new_pose.pose.orientation.x = number_list[3];
      new_pose.pose.orientation.y = number_list[4];
      new_pose.pose.orientation.z = number_list[5];
      new_pose.pose.orientation.w = number_list[6];
      global_waypoint_list_.push_back(new_pose);
//      Eigen::Vector3f pt = Eigen::Vector3f(number_list[0], number_list[1], number_list[2]);
//      Eigen::Quaternionf quat = Eigen::Quaternionf(number_list[6], number_list[3], number_list[4], number_list[5]);
      float angle = acos(number_list[6])*2.0;
      //std::cout << angle*360/M_PI << std::endl;
    }
    waypointFile.close();
    std::cout << "Way point file with " << global_waypoint_list_.size() << " way points has been successfully parsed." << std::endl;

    local_waypoint_list_ = getPath(0.0,
                                   0.0,
                                   0.0,
                                   global_waypoint_list_[current_global_waypoint_index_].pose.position.x,
                                   global_waypoint_list_[current_global_waypoint_index_].pose.position.y,
                                   acos(global_waypoint_list_[current_global_waypoint_index_].pose.orientation.w)*2.0);

  } else {
    std::cout << "Failed to parse way point file" << std::endl;
  }

}

GoalPositionUpdater::~GoalPositionUpdater() {
  // TODO Auto-generated destructor stub
}

void GoalPositionUpdater::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  last_goal_ = current_goal_;
  current_goal_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  ++goals_received_;
  std::cout << "New goal position received." << std::endl;
}

void GoalPositionUpdater::timer_callback(const ros::TimerEvent& e) {

  geometry_msgs::PoseStamped pose_msg = local_waypoint_list_[current_local_waypoint_index_];
  tf::Transform transform_auxiliar;
  transform_auxiliar.setOrigin(tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
  transform_auxiliar.setRotation(tf::Quaternion( pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w));
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform_auxiliar, ros::Time::now(), "/map", "/waypoint"));

  nav_msgs::Path local_path;
  local_path.header.frame_id="/map";
  local_path.poses = local_waypoint_list_;
  local_path_publisher_.publish(local_path);
  nav_msgs::Path global_path;
  global_path.header.frame_id="/map";
  global_path.poses = global_waypoint_list_;
  global_path_publisher_.publish(global_path);
  geometry_msgs::PoseStamped next_waypoint;
  next_waypoint.pose = local_waypoint_list_[current_local_waypoint_index_].pose;
  next_waypoint.header.frame_id = "/map";
  next_waypoint.header.stamp = ros::Time::now();
  goal_publisher_.publish(next_waypoint);

//  if (goals_received_ < 1) return;

  //obtain current location
  tf::StampedTransform transform_w_b;
  try{
    tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_w_b);
  }
  catch (tf::TransformException& ex){
    //std::cout << "pose not received" << std::endl;
    return;
    //ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }
  Eigen::Vector3d robot_pose = Eigen::Vector3d (transform_w_b.getOrigin().x(),transform_w_b.getOrigin().y(),transform_w_b.getOrigin().z());
  Eigen::Quaterniond robot_quat = Eigen::Quaterniond (transform_w_b.getRotation().getW(), transform_w_b.getRotation().getX(), transform_w_b.getRotation().getY(), transform_w_b.getRotation().getZ());

  int result = checkPosition(robot_pose(0),
                             robot_pose(1),
                             local_waypoint_list_[current_local_waypoint_index_].pose.position.x,
                             local_waypoint_list_[current_local_waypoint_index_].pose.position.y,
                             acos(local_waypoint_list_[current_local_waypoint_index_].pose.orientation.w)*2.0);

  if (result >= 1) { //advance to next local  waypoint
    ++current_local_waypoint_index_;
    if (current_local_waypoint_index_>=local_waypoint_list_.size()-1) { //advance to next global waypoint
      ++current_global_waypoint_index_;
      if(current_global_waypoint_index_>=global_waypoint_list_.size()){
        current_global_waypoint_index_ = 0;
      }

      local_waypoint_list_ = getPath(robot_pose(0),
                                     robot_pose(1),
                                     acos(robot_quat.w())*2.0,
                                     global_waypoint_list_[current_global_waypoint_index_].pose.position.x,
                                     global_waypoint_list_[current_global_waypoint_index_].pose.position.y,
                                     acos(global_waypoint_list_[current_global_waypoint_index_].pose.orientation.w)*2.0);
      current_local_waypoint_index_ = 1;
    }
  }

  //compute new angle/vel
  computeAndPublishNextCommand();
  //std::cout << "quat z comp " << robot_quat.z() << std::endl;
  //std::cout << "quat w comp " << robot_quat.w() << std::endl;
}

int GoalPositionUpdater::checkPosition(float xc_w, float yc_w, float Xway_w, float Yway_w, float Thway_w) {

  float distance = sqrt(pow(xc_w-Xway_w, 2)+pow(yc_w-Yway_w,2));
  std::cout << "distance: " << sqrt(pow(xc_w-Xway_w, 2)+pow(yc_w-Yway_w,2)) << std::endl;
  if (distance < 0.5)
    return 1;
  return 0;



  float a=cos(Thway_w*M_PI/180);
  float b=-sin(Thway_w*M_PI/180);
  float c=Xway_w;
  float d=sin(Thway_w*M_PI/180);
  float e=cos(Thway_w*M_PI/180);
  float f=Yway_w;

  float xnew=a*xc_w+b*yc_w+(-a*c-d*f)*1;
  float ynew=b*xc_w+e*yc_w+(-b*e-c*f)*1;

  int status;

  if (ynew > 0)
    status = 1; //load the next waypoint
  else status = 0;

/*
  float threshold_min = 0.04;
  float threshold_max = 0.25; //if out of this limit
  float diffx = xc - xg;
  float diffy = yc - yg;
  dist = sqrt(pow(diffx,2) + pow(diffy,2));
  if ((dist>threshold_min) && (dist<threshold_max))
    status = 0;
  else if (dist > threshold_max)
        status = 2;
  else status = 1;
*/
  return status;
}

void GoalPositionUpdater::computeAndPublishNextCommand() {
  tf::StampedTransform transform_r_way;
  try{
    tf_listener_.lookupTransform("/base_link", "/waypoint", ros::Time(0), transform_r_way);
  }
  catch (tf::TransformException& ex){
    return;
  }
  Eigen::Vector3d waypoint_pose = Eigen::Vector3d (transform_r_way.getOrigin().x(),transform_r_way.getOrigin().y(),transform_r_way.getOrigin().z());

  //std::cout << waypoint_pose << std::endl;
  float theta = atan2(waypoint_pose[1], waypoint_pose[0]);

  std_msgs::Float32 steer_cmd_msg;
  steer_cmd_msg.data=std::min(std::max(-theta*180/M_PI, -35.0), 35.0);

  std::cout << "Send angle: " << steer_cmd_msg.data << std::endl;
  steer_command_publisher_.publish(steer_cmd_msg); //publish message

}
