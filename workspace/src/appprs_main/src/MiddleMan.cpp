/*
 * MiddelMan.cpp
 *
 *  Created on: Sep 25, 2015
 *      Author: vgrabe
 */

#include "appprs_main/MiddleMan.h"

const size_t button = 0;
const size_t steer_axis = 0;
const size_t speed_axis = 1;
const float speed_factor = 1.0;
const float steer_factor = -23.0;
const float max_speed_acc = 0.1;
const float max_steer_acc = 2;

MiddleMan::MiddleMan():
  last_speed_(0.0), last_steer_(0.0){
  joy_subscriber_ = nh_.subscribe("/joy", 10, &MiddleMan::joystickCallback, this); //Read joystick Steer
  steer_command_subscriber_ = nh_.subscribe("/path_steer", 1, &MiddleMan::pathSteerCallback, this); //Read path planner Steer
  speed_command_subscriber_  = nh_.subscribe("/path_speed", 1, &MiddleMan::pathSpeedCallback, this); //Read path planner Speed
  steer_command_publisher_ = nh_.advertise<std_msgs::Float32>("steerCommand", 1); //create a publisher
  speed_command_publisher_ = nh_.advertise<std_msgs::Float32>("speedCommand", 1); //create a publisher
  watchdog_timer_ = nh_.createTimer(ros::Duration(0.1), &MiddleMan::timerCallback, this);
}

MiddleMan::~MiddleMan() {
  // TODO Auto-generated destructor stub
}

void MiddleMan::pathSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (last_gamepad_msg_.buttons.size()>button && last_gamepad_msg_.buttons[button]) {
    // nothing yet
  } else {
    sendSpeed(msg->data);
  }
}

void MiddleMan::pathSteerCallback(const std_msgs::Float32::ConstPtr& msg) {
  if(last_gamepad_msg_.buttons.size()>button && last_gamepad_msg_.buttons[button]){
    //nothing for now
  } else {
    sendSteer(msg->data);
  }
}

void MiddleMan::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  last_gamepad_msg_ = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "middle_man"); //Make listening node

  ros::AsyncSpinner spinner(1);
  MiddleMan *mm = new MiddleMan();
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();
  delete mm;

  return 0;
}

void MiddleMan::timerCallback(const ros::TimerEvent& e) {
  if (last_gamepad_msg_.buttons.size()>button && last_gamepad_msg_.buttons[button]) {
    float steer = 0;
    float speed = 0;
    if (last_gamepad_msg_.axes.size()>steer_axis) {
      steer = last_gamepad_msg_.axes[steer_axis]*steer_factor;
    }
    sendSteer(steer);

    if (last_gamepad_msg_.axes.size()>speed_axis) {
      speed = last_gamepad_msg_.axes[speed_axis]*speed_factor;
    }
    sendSpeed(speed);
  }
}

void MiddleMan::sendSpeed(float speed) {
  std_msgs::Float32 msg;
  float delta = speed-last_speed_;
  delta = std::min(std::max(delta, -max_speed_acc), max_speed_acc);
  msg.data = last_speed_+delta;
  speed_command_publisher_.publish(msg);
  last_speed_ = msg.data;
}

void MiddleMan::sendSteer(float steer) {
  std_msgs::Float32 msg;
  float delta = steer-last_steer_;
  delta = std::min(std::max(delta, -max_steer_acc), max_steer_acc);
  msg.data = last_steer_+delta;
  steer_command_publisher_.publish(msg);
  last_steer_ = msg.data;
}
