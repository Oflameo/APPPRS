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
// static_test_with_point_clouds.bag is publishing PointCloud on /my_cloud, not PointCloud2...
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //See tutorials section 4.2 here:  http://wiki.ros.org/pcl/Tutorials 
  //Need to create shared pointers here - required type for pass-through filters
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 output;  //output cloud in ROS format

  // Convert to pcl cloud format from ROS msg
  pcl::fromROSMsg (*input, *cloud);

  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*
  // Perform the actual filtering
  //This is what we actually want.
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-4.0, 4.0);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-3.5, 4.0);
  pass.filter (*cloud_filtered2);



  //std::cout << "PointCloud after filtering has: " << cloud_filtered2->points.size () << " data points." << std::endl; //*
  // Publish the data.
  pcl::toROSMsg(*cloud_filtered2, output);
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("my_cloud2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);

  // Spin
  ros::spin ();
}

