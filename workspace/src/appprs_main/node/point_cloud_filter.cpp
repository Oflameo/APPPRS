//The goal of this node is to take the full sensor_msgs::PointCloud2 created by the laser scans and filter it to remove the map PURELY based on geometry. This is not meant to do scan-matching and subtraction from the map.

//it reads from the my_cloud topic which is a point cloud in sensor_msgs::PointCloud2 form, then puts it into a pcl data structure so it can be filtered. When you create the filter though, it segfaults, even when you follow

//After the filtering it will transform the pcl structure back to a sensor_msgs::PointCloud2 for publishing


#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;


// Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered_y;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);


//TODO: fix this filter so it doesn't throw a segfault : (

  // Perform the actual filtering
  pcl::PassThrough<pcl::PointXYZ> pass; //This is just to check against the tutorial (http://pointclouds.org/documentation/tutorials/passthrough.php)

 // pcl::PassThrough<pcl::PCLPointCloud2> pass; //This is what we actually want.

  //pass_y.setInputCloud (cloudPtr);
  //pass_y.setFilterFieldName ("y");
  //pass_y.setFilterLimits (-4.0, 4.0);
  //pass_y.filter (cloud_filtered_y);
  // Convert to ROS data type
 // pcl_conversions::fromPCL(cloud_filtered_y, output);


//TODO: Change input back to output once the filtering is fixed.
  // Publish the data.
  pub.publish (input);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("my_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);

  // Spin
  ros::spin ();
}
