/*
 * image_read_test.cpp
 *
 *  Created on: Nov 14, 2015
 *      Author: James Gabriel
 */

#include "ros/ros.h"
#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/publisher.h>

using namespace cv;
using namespace std;

ros::Publisher pub;

//Declare Functions
void initialize_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat image);


int main(int argc,  char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl_tutorial");
ros::NodeHandle nh;
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

//Load the Map into Memory
cv::Mat image;
string imageName("/home/jamie/workspace/ConvertToImage/src/wean_map_uint8.jpg"); // by default
image=cv::imread(imageName,CV_LOAD_IMAGE_GRAYSCALE);

//Check that you got the image
if(!image.data )                            
{
    cout <<  "Could not open or find the image" << std::endl ;
    return -1;
}


//Create your Point Clouds Containers. One for minipulation, one for export #TPP
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 output;


//Set Cloud Size: TODO: Make this variable in some future version based on probabilities
(*cloud).width  = 1000;
(*cloud).height = 1;
(*cloud).points.resize ((*cloud).width * (*cloud).height);


//Initialize Points
initialize_points(cloud, image);

//Print that you are done to let you know you haven't gotten cought in some
 //sort of loop
 cout<<"done making points"<<endl;

//Convert between point cloud types	 
 pcl::toROSMsg(*cloud, output);
 
 //Set the reference frame for your pointcloud (ROS bookkeeping)
 output.header.frame_id = std::string("/odom");
 
//Set the loop rate for republishing points. This will cease to be necessary later on
 ros::Rate loop_rate(10);
 
 //Keep publishing points to RVIZ to keep the points on the screen
 while(nh.ok())
 {
output.header.stamp = ros::Time::now();
pub.publish (output);
ros::spinOnce ();
loop_rate.sleep();
 }
	return 0;
}


void initialize_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat image)
{
	 //Create your Points
	 for (size_t i = 0; i < (*cloud).points.size (); ++i)
	 {
		 bool isbad=0;
		 int  Count=0;
		 
		 do
		 {
			 //Start out with a random point within the sub-rectangle that I decided 
			 //heuristically was a good point
			 Count++;
			 int pix_x=rand() % 350+350;
			 int pix_y=rand() % 761;
	         (*cloud).points[i].x = (pix_x/10.0);
	         (*cloud).points[i].y = (pix_y/10.0);
	         (*cloud).points[i].z = 0;
		 
	         //Check what map value that point starts at using the map-image
	         int good=image.at<uchar>(800-pix_y,pix_x);
	   
	         //Only allow points to exist at places with high robot-probability
	         isbad=(good<250);

		 }while(isbad && (Count<1000));
	 
	 }
	 return;
}


