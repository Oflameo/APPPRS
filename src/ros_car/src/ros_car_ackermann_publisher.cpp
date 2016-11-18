/*
 * ros_car_ackermann_publisher.cpp
 *
 *  Created on: Sep 30, 2016
 *      Author: jd
 */


#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float32.h>

const float pi = 3.1415927;

const float wheel_base = 0.74295; // [m] Axle to axle distance
const float min_turn_radius = 0.1; // [m]

ros::Publisher ackermann_pub;
ros::Subscriber cmd_vel_sub;

ackermann_msgs::AckermannDrive ackermann_msg;

float base_speed_cmd_ = 0.0, base_omega_cmd_ = 0.0, steer_angle_cmd_ = 0.0, turn_radius_ = 0.0;

void publishAckermannMsg(void) {

    ackermann_msg.steering_angle = steer_angle_cmd_;
    ackermann_msg.speed = base_speed_cmd_;

    ackermann_pub.publish(ackermann_msg);

}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    base_speed_cmd_ = msg->linear.x;
    base_omega_cmd_ = msg->angular.z;

    turn_radius_ = base_speed_cmd_/base_omega_cmd_;

    steer_angle_cmd_ = 0.0;

    if (fabs(base_omega_cmd_) > 0 && fabs(turn_radius_) >= min_turn_radius) {

		steer_angle_cmd_ = atan(wheel_base/turn_radius_);

    } else {

    	steer_angle_cmd_ = 0.0;

    }

    steer_angle_cmd_ = steer_angle_cmd_*180.0/pi; // Convert from radians to degrees

    publishAckermannMsg( );

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "ros_car_ackermann_publisher");

  ros::NodeHandle n;

  ackermann_pub = n.advertise<ackermann_msgs::AckermannDrive>("cmd_car", 50);
  cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmdVelCallback);

  ros::Rate r(1000);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    r.sleep();

  }

}
