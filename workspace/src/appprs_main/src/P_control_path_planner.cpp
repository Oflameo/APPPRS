//PID controller

#include "ros/ros.h"
#include "appprs_main/PcntrlPP.h"

float PI=3.14159265;

bool add(appprs_main::PcntrlPP::Request  &req,
         appprs_main::PcntrlPP::Response &res)
{
/*
req.Xway_w
req.Yway_w
req.Thway_w
req.XRob_w
req.YRob_w
req.ThRob_w
float32 Th_steer_r
float32 Vel_r
*/

float a, b, c, d, e, f, xnew, ynew;
a=cos(req.ThRob_w*PI/180);
b=-sin(req.ThRob_w*PI/180);
c=req.XRob_w;
d=-sin(req.ThRob_w*PI/180);
e=cos(req.ThRob_w*PI/180);
f=req.YRob_w;


xnew=a*req.Xway_w+b*req.Yway_w+(-a*c-d*f)*1;
ynew=b*req.Xway_w+e*req.Yway_w+(-b*e-c*f)*1;
if(xnew)
{
res.Th_steer_r=atan(ynew/xnew)*180/PI;
}
else
{
res.Th_steer_r=0;
}

res.Vel_r=.25;


ROS_INFO("New Steer Angle is %f, New Velocity is %f", res.Th_steer_r, res.Vel_r);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "P_control_path_planner");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("P_control_path_planner", add);
  ROS_INFO("Ready to do robot shtuff");
  ros::spin();

  return 0;
}
