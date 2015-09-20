/*
 * GoalPositionUpdater.h
 *
 *  Created on: Aug 19, 2015
 *      Author: vgrabe
 */

#ifndef GOALPOSITIONUPDATER_H_
#define GOALPOSITIONUPDATER_H_

#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class GoalPositionUpdater {
public:
  GoalPositionUpdater();
  virtual ~GoalPositionUpdater();
  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void timer_callback(const ros::TimerEvent& e);
  inline Eigen::Vector3d getLastGoal() {return last_goal_;}
  inline Eigen::Vector3d getCurrentGoal() {return current_goal_;}
private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher goal_publisher_;
  ros::Publisher global_path_publisher_, local_path_publisher_;
  ros::Publisher steer_command_publisher_;
  ros::Publisher speed_command_publisher_;
  ros::Subscriber goal_subscriber_;
  Eigen::Vector3d last_goal_, current_goal_;
  ros::Timer timer_;
  double distance_threshold_;
  int goals_received_;
  std::vector<geometry_msgs::PoseStamped> global_waypoint_list_, local_waypoint_list_;
  size_t current_global_waypoint_index_, current_local_waypoint_index_;

  int checkPosition();
  void computeAndPublishNextCommand();

};

#endif /* GOALPOSITIONUPDATER_H_ */
