#include "ros/ros.h"
#include "time.h"
#include "std_msgs/Float32.h"


//#include "cmath.h"

const float dist_per_tick = 1.3522*0.5*0.001;//units are meters

//Instantiate  global variables
double PI=3.1415926535897932384;

//speed publishers
 // steer_command_publisher = n.advertise<std_msgs::Float32>("/steerCommand", 1);
  //speed_command_publisher = n.advertise<std_msgs::Float32>("/speedCommand", 1);


//Callback Function
void joystick(const std_msgs::Float32& joyspeed)
{

//std_msgs::Float32 steer_cmd_msg, speed_cmd_msg;
}


void pathSpeedCallback(const std_msgs::Float32& pathspeed)
{
 // std_msgs::Float32 steer_cmd_msg, speed_cmd_msg;
}

void pathSteerCallback(const std_msgs::Float32& pathsteer)
{

}


//Main Function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_speed"); //Make listening node
  ros::NodeHandle n;
  
  
ros::Subscriber joySteersub = n.subscribe("/joy", 10, joystick); //Read joystick Steer
ros::Subscriber pathSteersub = n.subscribe("/path_steer", 10, pathSteerCallback); //Read path planner Steer
ros::Subscriber pathSpeedsub = n.subscribe("/path_speed", 10, pathSpeedCallback); //Read path planner Speed

ros::Publisher steer_command_publisher = n.advertise<std_msgs::Float32>("steerCommand", 1000); //create a publisher
ros::Publisher speed_command_publisher = n.advertise<std_msgs::Float32>("speedCommand", 1000); //create a publisher


std_msgs::Float32 steer_cmd_msg; //Create the message
std_msgs::Float32 speed_cmd_msg; //Create the message

steer_command_publisher.publish(steer_cmd_msg); //publish message
speed_command_publisher.publish(speed_cmd_msg); //publish message




ros::spin(); //spin me right round baby right round

  return 0;
}

