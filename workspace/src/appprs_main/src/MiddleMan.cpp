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
const float steer_factor = 20.0;

MiddleMan::MiddleMan() {
  ros::Subscriber joy_subscriber_ = nh_.subscribe("/joy", 10, &MiddleMan::joystickCallback, this); //Read joystick Steer
  ros::Subscriber steer_command_subscriber_ = nh_.subscribe("/path_steer", 1, &MiddleMan::pathSteerCallback, this); //Read path planner Steer
  ros::Subscriber speed_command_subscriber_  = nh_.subscribe("/path_speed", 1, &MiddleMan::pathSpeedCallback, this); //Read path planner Speed
  ros::Publisher steer_command_publisher_ = nh_.advertise<std_msgs::Float32>("steerCommand", 1); //create a publisher
  ros::Publisher speed_command_publisher_ = nh_.advertise<std_msgs::Float32>("speedCommand", 1); //create a publisher

}

MiddleMan::~MiddleMan() {
  // TODO Auto-generated destructor stub
}

void MiddleMan::pathSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (last_gamepad_msg_.buttons.size()>button && last_gamepad_msg_.buttons[button]) {
    std_msgs::Float32 out_msg;
    out_msg.data = 0;
    if (last_gamepad_msg_.axes.size()>speed_axis) {
      std_msgs::Float32 out_msg;
      out_msg.data = last_gamepad_msg_.axes[speed_axis]*speed_factor;
    }
    speed_command_publisher_.publish(out_msg);
  } else {
    speed_command_publisher_.publish(msg);
  }
}

void MiddleMan::pathSteerCallback(const std_msgs::Float32::ConstPtr& msg) {
  if(last_gamepad_msg_.buttons.size()>button && last_gamepad_msg_.buttons[button]){
    std_msgs::Float32 out_msg;
    out_msg.data = 0;
    if (last_gamepad_msg_.axes.size()>steer_axis) {
      std_msgs::Float32 out_msg;
      out_msg.data = last_gamepad_msg_.axes[steer_axis]*steer_factor;
    }
    steer_command_publisher_.publish(out_msg);
  } else {
    steer_command_publisher_.publish(msg);
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
