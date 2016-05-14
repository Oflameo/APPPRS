#include "ros/ros.h"
#include <math.h>
//#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "time.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "steerCommand");//start node
	ros::NodeHandle n;//create handle
	//  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("carCommand", 1000); //create a publisher
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("/steerCommand", 1); //create a publisher
	ros::Rate loop_rate(50); //set goal refresh rate (hz)


	int count=0;
	while (ros::ok())
	{
		//    std_msgs::Float32MultiArray cmd_msg; //Create the message
		std_msgs::Float32 steer_cmd_msg; //Create the message
		//cmd_msg.data[1] = (float) 15*sin(3*ros::Time::now().toSec()); //Calculate angle
		steer_cmd_msg.data = (float) 10*sin(3*ros::Time::now().toSec()); //Calculate angle






		chatter_pub.publish(steer_cmd_msg); //publish message

		ros::spinOnce();//Spin me right round

		loop_rate.sleep(); //Sleep to hit goal refresh rate
	}


	return 0;
}



