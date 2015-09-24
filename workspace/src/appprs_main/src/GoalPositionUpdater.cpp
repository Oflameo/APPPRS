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
#include <sensor_msgs/Joy.h>
#include "appprs_main/GoalPositionUpdater.h"
#include "appprs_main/localPP.h"
#include "appprs_main/p_control_path_planner.h"

GoalPositionUpdater::GoalPositionUpdater():
goals_received_(0), current_global_waypoint_index_(0), current_local_waypoint_index_(1){
  local_path_publisher_  = nh_.advertise<nav_msgs::Path>("/local_path", 10);
  global_path_publisher_ = nh_.advertise<nav_msgs::Path>("/global_path", 10);
  goal_publisher_  = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal2", 10);
  steer_command_publisher_ = nh_.advertise<std_msgs::Float32>("/steerCommand", 1);
  speed_command_publisher_ = nh_.advertise<std_msgs::Float32>("/speedCommand", 1);
  goal_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GoalPositionUpdater::goal_callback, this);
  joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &GoalPositionUpdater::joy_callback, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &GoalPositionUpdater::timer_callback, this);
  ros::NodeHandle pnh("~");
  std::string waypoint_file_name;
  pnh.param("waypoint_file", waypoint_file_name, std::string("/tmp/waypoints.txt"));
  distance_threshold_ = 1.0;

  ROS_INFO_STREAM("Attempting to open waypoint file " << waypoint_file_name << ".");
  std::ifstream waypointFile (waypoint_file_name);
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
        ROS_FATAL_STREAM("Unable to parse way point list, less than 7 tokens in line "<< line << "!");
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
    }
    waypointFile.close();
    ROS_INFO_STREAM("Way point file with " << global_waypoint_list_.size() << " way points has been successfully parsed.");

    local_waypoint_list_ = getPath(0.0,
                                   0.0,
                                   0.0,
                                   global_waypoint_list_[current_global_waypoint_index_].pose.position.x,
                                   global_waypoint_list_[current_global_waypoint_index_].pose.position.y,
                                   acos(global_waypoint_list_[current_global_waypoint_index_].pose.orientation.w)*2.0);

  } else {
    ROS_FATAL_STREAM("Failed to parse way point file");
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

  int result = checkPosition();

  if (result >= 1) { //advance to next local  waypoint
    ++current_local_waypoint_index_;
    if (current_local_waypoint_index_>=local_waypoint_list_.size()-2) { //advance to next global waypoint
      ++current_global_waypoint_index_;
      if(current_global_waypoint_index_>=global_waypoint_list_.size()){
        current_global_waypoint_index_ = 0;
      }

      //std::cout << "robot: " << robot_quat.toRotationMatrix().eulerAngles(0, 1, 2)*180/M_PI << ", " << acos(robot_quat.w())*2.0*180/M_PI << std::endl;
      Eigen::Quaterniond waypoint_quat(global_waypoint_list_[current_global_waypoint_index_].pose.orientation.w,
                                       global_waypoint_list_[current_global_waypoint_index_].pose.orientation.x,
                                       global_waypoint_list_[current_global_waypoint_index_].pose.orientation.y,
                                       global_waypoint_list_[current_global_waypoint_index_].pose.orientation.z);
      //std::cout << "map: " << waypoint_quat.toRotationMatrix().eulerAngles(0, 1, 2)*180/M_PI << ", " << acos(global_waypoint_list_[current_global_waypoint_index_].pose.orientation.w)*2.0*180/M_PI << std::endl;

      local_waypoint_list_ = getPath(robot_pose(0),
                                     robot_pose(1),
                                     robot_quat.toRotationMatrix().eulerAngles(0, 1, 2)(2),
                                     global_waypoint_list_[current_global_waypoint_index_].pose.position.x,
                                     global_waypoint_list_[current_global_waypoint_index_].pose.position.y,
                                     waypoint_quat.toRotationMatrix().eulerAngles(0, 1, 2)(2));
      current_local_waypoint_index_ = 1;
    }
  }

  //compute new angle/vel
  computeAndPublishNextCommand();
  //std::cout << "quat z comp " << robot_quat.z() << std::endl;
  //std::cout << "quat w comp " << robot_quat.w() << std::endl;
}

int GoalPositionUpdater::checkPosition() {

  tf::StampedTransform transform_r_base;
  try{
    tf_listener_.lookupTransform("/waypoint","/base_link", ros::Time(0), transform_r_base);
  }
  catch (tf::TransformException& ex){
    return 0;
  }
  Eigen::Vector3d base_pose = Eigen::Vector3d (transform_r_base.getOrigin().x(),transform_r_base.getOrigin().y(),transform_r_base.getOrigin().z());

  if (base_pose[0] > -0.5)
    return 1;
  return 0;

}

void GoalPositionUpdater::joy_callback(const sensor_msgs::Joy::ConstPtr& msg) {
  if(msg->buttons.size() > 5 &&  msg->buttons[5] == 1 && last_joy_msg_.buttons[5] == 0){

  }
  if(msg->buttons.size() > 6 && msg->buttons[6] == 1 && last_joy_msg_.buttons[6] == 0){

  }
  last_joy_msg_ = msg;
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

  std_msgs::Float32 steer_cmd_msg, speed_cmd_msg;
  steer_cmd_msg.data=std::min(std::max(-theta*180/M_PI, -35.0), 35.0);
  speed_cmd_msg.data=(40.0-fabs(steer_cmd_msg.data))*(1/40.0); //between 0.125 at 35deg and 1 at 0 deg

  ROS_DEBUG_STREAM("Send angle: " << steer_cmd_msg.data << ", velocity: " << speed_cmd_msg.data);
  steer_command_publisher_.publish(steer_cmd_msg); //publish message
  speed_command_publisher_.publish(speed_cmd_msg); //publish message

  //std::cout << "Send speed: " << speed_cmd_msg.data << std::endl;
  speed_command_publisher_.publish(speed_cmd_msg); //publish message

 

}
