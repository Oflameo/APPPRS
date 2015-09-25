/*
 * MiddelMan.h
 *
 *  Created on: Sep 25, 2015
 *      Author: vgrabe
 */

#ifndef MIDDELMAN_H_
#define MIDDELMAN_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

class MiddleMan {
public:
  MiddleMan();
  virtual ~MiddleMan();

private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_subscriber_;
  ros::Subscriber steer_command_subscriber_;
  ros::Subscriber speed_command_subscriber_;
  ros::Publisher steer_command_publisher_;
  ros::Publisher speed_command_publisher_;

  void pathSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
  void pathSteerCallback(const std_msgs::Float32::ConstPtr& msg);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);

  sensor_msgs::Joy last_gamepad_msg_;
};

#endif /* MIDDELMAN_H_ */
