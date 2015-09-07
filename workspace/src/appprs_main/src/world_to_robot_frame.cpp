#include "ros/ros.h"
#include "appprs_main/worldToRobot.h"

float PI=3.14159265;

bool add(appprs_main::worldToRobot::Request  &req,
         appprs_main::worldToRobot::Response &res)
{


/*
req.Xway_w
req.Yway_w
req.Thway_w
req.XRob_w
req.YRob_w
req.ThRob_w
*/

float a, b, c, d, e, f, xnew, ynew;
a=cos(req.ThRob_w*PI/180);
b=-sin(req.ThRob_w*PI/180);
c=req.XRob_w;
d=sin(req.ThRob_w*PI/180);
e=cos(req.ThRob_w*PI/180);
f=req.YRob_w;

ROS_INFO("A=%f, B=%f, C=%f, D=%f, E=%f, F=%f", a,b,c,d,e,f);


xnew=a*req.Xway_w+b*req.Yway_w+(-a*c-d*f)*1;
ynew=b*req.Xway_w+e*req.Yway_w+(-b*e-c*f)*1;
ROS_INFO("xnew=%f, ynew=%f", xnew, ynew);

 if(ynew>abs(xnew))
 {
   res.Safe = true;
 }  
 else
 {
   res.Safe=false;
 }

  if (res.Safe)
  {
	ROS_INFO("You Are Good");
  }
  
  else
  {
  	ROS_INFO("You are screwed");
  }
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_to_robot_frame");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("world_to_robot_frame", add);
  ROS_INFO("Ready to do robot shtuff");
  ros::spin();

  return 0;
}
