#include "ros/ros.h"
//#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "time.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("servo", 1000);

  ros::Rate loop_rate(50); //Publish at 50 hz
 
  while (ros::ok())
  {
    std_msgs::Float32 cmd_msg;
    cmd_msg.data = (float) 15*sin(3*ros::Time::now().toSec());

    chatter_pub.publish(cmd_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
