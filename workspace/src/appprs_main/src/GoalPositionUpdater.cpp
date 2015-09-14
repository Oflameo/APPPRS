/*
 * GoalPositionUpdater.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: vgrabe
 */

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
  new_command_publisher_ = nh_.advertise<std_msgs::Float32>("/carCommand", 1000);
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
  tf::StampedTransform transform;
  try{
//    tf_listener_.lookupTransform("/base_link", "/map", ros::Time(0), transform);
    tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException& ex){
    //std::cout << "pose not received" << std::endl;
    return;
    //ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }
  Eigen::Vector3d robot_pose = Eigen::Vector3d (transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
  Eigen::Quaterniond robot_quat = Eigen::Quaterniond (transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());

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
  computeAndPublishNextCommand(robot_pose(0),
                               robot_pose(1),
                               acos(robot_quat.w())*2.0*-robot_quat.z()/fabs(robot_quat.z()),
                               local_waypoint_list_[current_local_waypoint_index_].pose.position.x,
                               local_waypoint_list_[current_local_waypoint_index_].pose.position.y,
                               acos(local_waypoint_list_[current_local_waypoint_index_].pose.orientation.w)*2.0);
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

void GoalPositionUpdater::computeAndPublishNextCommand(float XRob_w,
    float YRob_w, float ThRob_w, float Xway_w, float Yway_w, float Thway_w) {

  float a, b, c, d, e,f;
  float xway_r, yway_r, Th_steer_r;
  float Vel_r;

  a=cos(ThRob_w);
  b=-sin(ThRob_w);
  c=XRob_w;
  d=sin(ThRob_w);
  e=cos(ThRob_w);
  f=YRob_w;

//  a=cos(ThRob_w*M_PI/180);
//  b=-sin(ThRob_w*M_PI/180);
//  c=XRob_w;
//  d=sin(ThRob_w*M_PI/180);
//  e=cos(ThRob_w*M_PI/180);
//  f=YRob_w;


  xway_r=a*Xway_w+b*Yway_w+(-a*c-d*f)*1;
  yway_r=b*Xway_w+e*Yway_w+(-b*e-c*f)*1;


  //Th_steer_r=atan2(yway_r,xway_r)*180.0/M_PI;
  float Th_to_waypoint = atan2(Yway_w-YRob_w, Xway_w-XRob_w);
  Th_steer_r = Th_to_waypoint-ThRob_w;

  std::cout << "angle to waypoint " << Th_to_waypoint*180/M_PI << std::endl;

//  if(abs(xway_r)<.1) {
//    Th_steer_r=0;
//  }

  Vel_r=.25;

  ROS_INFO("New Steer Angle is %f, New Velocity is %f", Th_steer_r*180/M_PI, Vel_r); //Vel_r);

  std_msgs::Float32 cmd_msg;
  cmd_msg.data=Th_steer_r*180/M_PI;
  new_command_publisher_.publish(cmd_msg); //publish message

}
