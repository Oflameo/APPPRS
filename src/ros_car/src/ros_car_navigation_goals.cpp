
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

void define_waypoints(std::vector<move_base_msgs::MoveBaseGoal> &waypoints);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {

	double      close_enough_ = 1;
	double      goal_timeout_ = 30;
	std::string robot_frame_ = "base_link";
	std::string world_frame_ = "map";

  ros::init(argc, argv, "ros_car_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base",true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)) && (ros::ok() == true)) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  std::vector<move_base_msgs::MoveBaseGoal> waypoints;
  define_waypoints(waypoints);

  tf::TransformListener tf_listener_;

	int i = 0;

	waypoints[i].target_pose.header.frame_id = "map";
	waypoints[i].target_pose.header.stamp = ros::Time::now();

	ROS_INFO("Sending start goal");
    ac.sendGoal(waypoints[i]);

  while ( i < waypoints.size() ){

      if (i < (waypoints.size()-1)) {

			tf::StampedTransform robot_gb, goal_gb;

			try {
				tf_listener_.waitForTransform(world_frame_, robot_frame_, ros::Time(0), ros::Duration(5.0));
				tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0), robot_gb);
			} catch (tf::TransformException& e) {
				ROS_WARN("Cannot get tf %s -> %s: %s", world_frame_.c_str(), robot_frame_.c_str(), e.what());
				continue;
			}

			mtk::pose2tf(waypoints[i].target_pose, goal_gb);

			double distance = mtk::distance2D(robot_gb, goal_gb);

			if (distance <= close_enough_) {

				ROS_INFO("Close enough to current goal (%.2f <= %.2f m).", distance, close_enough_);

				i++;

				//we'll send a goal to the robot to move 1 meter forward
				waypoints[i].target_pose.header.frame_id = "map";
				waypoints[i].target_pose.header.stamp = ros::Time(0); // ros::Time::now();

				ROS_INFO("Sending next goal");
				ac.sendGoal(waypoints[i]);
			}

	  }

  }

  return 0;

}

void define_waypoints(std::vector<move_base_msgs::MoveBaseGoal> &waypoints){

	move_base_msgs::MoveBaseGoal goal;

//	goal.target_pose.pose.position.x = -0.302651862391;
//	goal.target_pose.pose.position.y = 10.4889070721;
//	goal.target_pose.pose.position.z = 0.0;
//
//	goal.target_pose.pose.orientation.x = 0.0;
//	goal.target_pose.pose.orientation.y = 0.0;
//	goal.target_pose.pose.orientation.z = 0.712574585931;
//	goal.target_pose.pose.orientation.w = 0.701596365075;
//
//	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = 7.78187570026;
	goal.target_pose.pose.position.y = 21.0280356809;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0233064853229;
	goal.target_pose.pose.orientation.w = 0.999728366979;

	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = 16.3385057892;
	goal.target_pose.pose.position.y = 14.6357488233;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = -0.707258210551;
	goal.target_pose.pose.orientation.w = 0.706955319386;

	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = 22.1503593149;
	goal.target_pose.pose.position.y = 8.33251734311;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0253035066246;
	goal.target_pose.pose.orientation.w = 0.999679815017;

	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = 28.1512263771;
	goal.target_pose.pose.position.y = -2.73225448067;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = -0.70727991869;
	goal.target_pose.pose.orientation.w = 0.706933601279;

	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = 21.0285301489;
	goal.target_pose.pose.position.y = -12.6753483517;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.883681901289;
	goal.target_pose.pose.orientation.w = 0.468087916245;

	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = 10.8175309567;
	goal.target_pose.pose.position.y = -4.74586151989;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = -0.853464689791;
	goal.target_pose.pose.orientation.w = 0.521150672339;

	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = -0.187200169267;
	goal.target_pose.pose.position.y = -6.66704815296;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.70452443672;
	goal.target_pose.pose.orientation.w = 0.709679729219;

	waypoints.push_back(goal);

	goal.target_pose.pose.position.x = -0.264290612115;
	goal.target_pose.pose.position.y = 1.21456405889;
	goal.target_pose.pose.position.z = 0.0;

	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.695591970233;
	goal.target_pose.pose.orientation.w = 0.718437061229;

	waypoints.push_back(goal);

}
