/*
 * GoalPositionUpdater.h
 *
 *  Created on: Aug 19, 2015
 *      Author: vgrabe
 */

#ifndef GOALPOSITIONUPDATER_H_
#define GOALPOSITIONUPDATER_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>
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
  ros::Publisher goal_publisher_;
  ros::Subscriber goal_subscriber_;
  Eigen::Vector3d last_goal_, current_goal_;
  ros::Timer timer_;
  double distance_threshold_;
  int goals_received_;
};

#endif /* GOALPOSITIONUPDATER_H_ */
