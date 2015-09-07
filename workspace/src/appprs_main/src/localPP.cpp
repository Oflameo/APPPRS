/*
Written by James Gabriel 9/5/2015

This program creates a seventh degree polynomial spline between your beginning point and destination point. Here, because we are working in the robot frame, that beginning point is always (0,0,0) [x,y,theta]. The final point can be an arbitrary point and direction.

Inputs: 3 floats representing x,y,theta of the destination point.

Outputs: Two arrays of "numWays" points (defined ~ln77) representing the x and y locations of subwaypoints along that seventh degree spline.


Math is all of my own derrivation and brilliant engineering mind
....psych no. It is from:
Piazzi, Aurelio, Corrado Guarino Lo Bianco, and Massimo Romano. 
"-Splines for the Smooth Path Generation of Wheeled Mobile Robots." 
Robotics, IEEE Transactions on 23.5 (2007): 1089-1095.
*/

#include "ros/ros.h"
#include "appprs_main/localPP_SRV.h"
#include <std_msgs/Float32MultiArray.h>


float PI=3.14159265;

bool add(appprs_main::localPP_SRV::Request  &req,
         appprs_main::localPP_SRV::Response &res)
{

float xa, ya, Tha, ka, kda;
float xb, yb, Thb, kb, kdb;
float a0, a1, a2, a3, a4, a5, a6, a7;
float b0, b1, b2, b3, b4, b5, b6, b7;

xa=0;                    //X coord starting point
ya=0;                    //Y coord starting point
Tha=90*PI/180;           //Angle of starting point
ka=0;                    //Desired curvature at the start We can change this to improve things
kda=.5;                  //Desired derivative of curvature at start...its effects are strange

xb=req.Xway_r;                         //set new waypoint
yb=req.Yway_r;                         //set new waypoint
Thb=req.Thway_r*PI/180;                //set new waypoint
kb=0;                                //desired end curvature
kdb=0;                              //derivative of curvature at end


float n[6]={1.5,1.5,0,0,0,0};  //These are the shape factors which will change the global spline characteristics


//x coordinate coefficients for parametric spline
    a0=xa;
    a1=n[0]*cos(Tha);
    a2=1/2*n[2]*cos(Tha)-1/2*pow(n[0],2)*ka*sin(Tha);
    a3=1/6*n[4]*cos(Tha)-1/6*(pow(n[0],3)*kda+3*n[0]*n[2]*ka)*sin(Tha);
    a4=35*(xb-xa)-(20*n[0]+5*n[2]+2/3*n[4])*cos(Tha)+(5*pow(n[0],2)*ka+2/3*pow(n[0],3)*kda+2*n[0]*n[2]*ka)*sin(Tha)-(15*n[1]-5/2*n[3]+1/6*n[5])*cos(Thb)-(5/2*pow(n[1],2)*kb-1/6*pow(n[1],3)*kdb-1/2*n[1]*n[3]*kb)*sin(Thb);
   
    a5=-84*(xb-xa)+(45*n[0]+10*n[2]+n[4])*cos(Tha)-(10*pow(n[0],2)*ka+pow(n[0],3)*kda+3*n[0]*n[2]*ka)*sin(Tha)+(39*n[1]-7*n[3]+1/2*n[5])*cos(Thb)+(7*pow(n[1],2)*kb-1/2*pow(n[1],3)*kdb-3/2*n[1]*n[3]*kb)*sin(Thb);
   
    a6=70*(xb-xa)-(36*n[0]+15/2*n[2]+2/3*n[4])*cos(Tha)+(15/2*pow(n[0],2)*ka+2/3*pow(n[0],3)*kda+2*n[0]*n[2]*ka)*sin(Tha)-(34*n[1]-13/2*n[3]+1/2*n[5])*cos(Thb)-(13/2*pow(n[1],2)*kb-1/2*pow(n[1],3)*kdb-3/2*n[1]*n[3]*kb)*sin(Thb);
    
    a7=-20*(xb-xa)+(10*n[0]+2*n[2]+1/6*n[4])*cos(Tha)-(2*pow(n[0],2)*ka+1/6*pow(n[0],3)*kda+1/2*n[0]*n[2]*ka)*sin(Tha)+(10*n[1]-2*n[3]+1/6*n[5])*cos(Thb)+(2*pow(n[1],2)*kb-1/6*pow(n[1],3)*kdb-1/2*n[1]*n[3]*kb)*sin(Thb);
    
    
    
//x coordinate coefficients for parametric spline
    b0=ya;
    b1=n[0]*sin(Tha);
    b2=1/2*n[2]*sin(Tha)+1/2*pow(n[0],2)*ka*cos(Tha);
    b3=1/6*n[4]*sin(Tha)+1/6*(pow(n[0],3)*kda+3*n[0]*n[2]*ka)*cos(Tha);
    b4=35*(yb-ya)-(20*n[0]+5*n[2]+2/3*n[4])*sin(Tha)-(5*pow(n[0],2)*ka+2/3*pow(n[0],3)*kda+2*n[0]*n[2]*ka)*cos(Tha)-(15*n[1]-5/2*n[3]+1/6*n[5])*sin(Thb)+(5/2*pow(n[1],2)*kb-1/6*pow(n[1],3)*kdb-1/2*n[1]*n[3]*kb)*cos(Thb);
    
    b5=-84*(yb-ya)+(45*n[0]+10*n[2]+n[4])*sin(Tha)+(10*pow(n[0],2)*ka+pow(n[0],3)*kda+3*n[0]*n[2]*ka)*cos(Tha)+(39*n[1]-7*n[3]+1/2*n[5])*sin(Thb)-(7*pow(n[1],2)*kb-1/2*pow(n[1],3)*kdb-3/2*n[1]*n[3]*kb)*cos(Thb);
    
    b6=70*(yb-ya)-(36*n[0]+15/2*n[2]+2/3*n[4])*sin(Tha)-(15/2*pow(n[0],2)*ka+2/3*pow(n[0],3)*kda+2*n[0]*n[2]*ka)*cos(Tha)-(34*n[1]-13/2*n[3]+1/2*n[5])*sin(Thb)+(13/2*pow(n[1],2)*kb-1/2*pow(n[1],3)*kdb-3/2*n[1]*n[3]*kb)*cos(Thb);

    b7=-20*(yb-ya)+(10*n[0]+2*n[2]+1/6*n[4])*sin(Tha)+(2*pow(n[0],2)*ka+1/6*pow(n[0],3)*kda+1/2*n[0]*n[2]*ka)*cos(Tha)+(10*n[1]-2*n[3]+1/6*n[5])*sin(Thb)-(2*pow(n[1],2)*kb-1/6*pow(n[1],3)*kdb-1/2*n[1]*n[3]*kb)*cos(Thb);



int numWays=10;                     //how many subdivions will be in my spline
std_msgs::Float32MultiArray px;     //multi array of x points
std_msgs::Float32MultiArray py;     //multi array of y points
for(int i=0; i<numWays; i++)
{
   float u=i/numWays;
   px.data[i]=a0+a1*u+a2*pow(u,2)+a3*pow(u,3)+a4*pow(u,4)+a5*pow(u,5)+a6*pow(u,6)+a7*pow(u,7);
   py.data[i]=b0+a1*u+b2*pow(u,2)+b3*pow(u,3)+b4*pow(u,4)+b5*pow(u,5)+b6*pow(u,6)+b7*pow(u,7);
}

res.pathX=px;                       //send sub-waypoints back to the main program
res.pathY=py;                       //send sub-waypoints back to the main program

  return true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "localPP");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("localPP", add);
  ROS_INFO("Local Path Planner Ready");
  ros::spin();

  return 0;
}
