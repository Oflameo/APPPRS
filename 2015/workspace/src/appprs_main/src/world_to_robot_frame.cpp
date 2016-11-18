#include "ros/ros.h"
//#include "appprs_main/worldToRobot.h"

bool safeWaypoint(float XRob_w, float YRob_w, float ThRob_w, float Xway_w, float Yway_w, float Thway_w, float& Vel_r, float& Th_steer_r){


float PI=3.14159265;
bool Safe;
float a, b, c, d, e, f, xnew, ynew;


a=cos(ThRob_w*PI/180);
b=-sin(ThRob_w*PI/180);
c=XRob_w;
d=sin(ThRob_w*PI/180);
e=cos(ThRob_w*PI/180);
f=YRob_w;

ROS_INFO("A=%f, B=%f, C=%f, D=%f, E=%f, F=%f", a,b,c,d,e,f);


xnew=a*Xway_w+b*Yway_w+(-a*c-d*f)*1;
ynew=b*Xway_w+e*Yway_w+(-b*e-c*f)*1;
ROS_INFO("xnew=%f, ynew=%f", xnew, ynew);

 if(ynew>abs(xnew))
 {
   Safe = true;
 }  
 else
 {
   Safe=false;
 }

  if (Safe)
  {
	ROS_INFO("You Are Good");
  }
  
  else
  {
  	ROS_INFO("You are screwed");
  }
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return Safe;
}

/*int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_to_robot_frame");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("world_to_robot_frame", add);
  ROS_INFO("Ready to do robot shtuff");
  ros::spin();

  return 0;
}*/
