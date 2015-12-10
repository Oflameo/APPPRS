#include "ros/ros.h"
#include "appprs_main/ClassifyLegs.h"
#include <cstdlib>

ros::ServiceClient* clientptr;

void callservice(int x){

  appprs_main::ClassifyLegs srv;
  srv.request.features = x;
  if (clientptr->call(srv))
  {
    ROS_INFO("label: %ld", (long int)srv.response.label);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "classify_legs_client");
  if (argc != 2)
  {
    ROS_INFO("usage: classify_legs_client X");
    return 1;
  }

  ros::NodeHandle n;
  //ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  ros::ServiceClient client = n.serviceClient<
      appprs_main::ClassifyLegs>("classify_legs");
  
  clientptr = &client;

  callservice(atoll(argv[1]));
  return 0;
}