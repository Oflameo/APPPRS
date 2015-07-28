#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "time.h"
#include <tf/transform_broadcaster.h>
//#include "cmath.h"


//Instantiate  global variables
double g_x=0, g_y=0;
double g_yaw=0, g_pitch=0, g_roll=0;
double gph=0, g_oldTime=0,totalDist=0;
double PI=3.1415926535897932384;


double cnvGyX(int GyX); //function prototype
double cnvGyY(int GyY); //function prototype
double cnvGyZ(int GyZ); //function prototype


//Callback Function
void chatterCallback(const std_msgs::Int16MultiArray& test)
{

  double newTime = ros::Time::now().toSec(); //current time

  g_yaw=g_yaw+(cnvGyZ(test.data[3])*(newTime-g_oldTime)); //discreet integration to calc yaw
  g_pitch+=cnvGyX(test.data[1])*(newTime-g_oldTime); //discreet integration to calc pitch
  g_roll+=cnvGyY(test.data[2])*(newTime-g_oldTime);  //discreet integration to calc roll

  g_x+=cos(g_yaw*PI/180)*test.data[4]*1.1*1/1000; //units are meters
  g_y+=sin(g_yaw*PI/180)*test.data[4]*1.1*1/1000; //units are meters

  ROS_INFO("X=%f, Y=%f, Yaw = %f degrees", g_x, g_y,g_yaw); //print results
  g_oldTime=newTime; //record the time
}


//Main Function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener"); //Make listening node
  ros::NodeHandle n;
  ros::init(argc, argv, "my_tf_broadcaster"); //Make broadcasting node
  ros::NodeHandle node;


  if (g_oldTime==0)
  {
  g_oldTime=ros::Time::now().toSec(); //Because ROS
  }


  ros::Subscriber sub = n.subscribe("testDat", 1000, chatterCallback); //Read Arduino
 
  tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(
    tf::StampedTransform(
       tf::Transform(tf::Quaternion(g_pitch, g_roll, g_yaw, 1), tf::Vector3(g_x, g_y, 0)),
         ros::Time::now(),"world", "IMU")); //Transmit TF

  ros::spin(); //spin me right round baby right round

  return 0;
}


//Conversion Functions

double cnvGyX(int GyX)
{
return ((double)GyX+353.16)*250/32768;
}

double cnvGyY(int GyY)
{
return ((double)GyY-65.41)*250/32768;
}

double cnvGyZ(int GyZ)
{
return ((double)GyZ-61.26)*250/32768;
}

