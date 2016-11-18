//PID controller

#include <math.h>
#include "ros/ros.h"
#include "appprs_main/p_control_path_planner.h"

bool computeNewAngleSpeed(float XRob_w, float YRob_w, float ThRob_w, float Xway_w, float Yway_w, float Thway_w, float& Vel_r, float& Th_steer_r){

  float a, b, c, d, e, f, xnew, ynew;
  a=cos(ThRob_w*M_PI/180);
  b=-sin(ThRob_w*M_PI/180);
  c=XRob_w;
  d=sin(ThRob_w*M_PI/180);
  e=cos(ThRob_w*M_PI/180);
  f=YRob_w;


  xnew=a*Xway_w+b*Yway_w+(-a*c-d*f)*1;
  ynew=b*Xway_w+e*Yway_w+(-b*e-c*f)*1;


  if(xnew) {
    Th_steer_r=atan(ynew/xnew)*180/M_PI;
  }
  else {
    Th_steer_r=0;
  }

  if(abs(xnew)<.1)
  {
    Th_steer_r=0; 
  }

  Vel_r=.25;


  ROS_INFO("New Steer Angle is %f, New Velocity is %f", Th_steer_r, Vel_r);

  return true;
}
