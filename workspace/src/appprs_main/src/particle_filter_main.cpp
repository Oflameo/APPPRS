/*
 * image_read_test.cpp
 *
 *  Created on: Nov 14, 2015
 *      Author: James Gabriel
 */

/*The purpose of this file is to build a particle filter. It does this by first
 importing a map which is a picture (I can send you that). Then,
 it creates a bunch of particles within that map for visualization in rviz. That part works
 just fine.

  What broke everything is when I tried to implement my
 "single_particle" class from "singleparticle.cpp" and "includes/appprs_main/singleparticle.h". This is just supposed
 to be a container for all the particle data with some simple particle functions. It has refused to work though and I am
 getting so irritated.

 The function where I try to make the particle objects is down at the bottom
 */

#include <appprs_main/singleparticle.h>
#include <appprs_main/particlefilter.h>

#include <ros/publisher.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <appprs_main/singleparticle.h>
#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <thread>
#include <chrono>

using namespace cv;
using namespace std;

ros::Publisher pub;

//Declare Functions

void temp_initialize_points(pcl::PointCloud<pcl::PointXYZ> &cloud,
		cv::Mat image,
		std::vector<boost::shared_ptr<single_particle>> &ptr_container,
		sensor_msgs::PointCloud2 &output);

void updateVisualization(ParticleFilter &pf,
		pcl::PointCloud<pcl::PointXYZ> &cloud,
        sensor_msgs::PointCloud2 &output,
		ros::NodeHandle nh);


int main(int argc,  char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    //std::vector<boost::shared_ptr<single_particle>> temp_ptr_container;

	//Create your Point Clouds Containers. One for minipulation, one for export #TPP
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	sensor_msgs::PointCloud2 output;

	//Instantiate Particle Filter
	ParticleFilter pf=ParticleFilter();

	//Set Cloud Size: TODO: Make this variable in some future version based on probabilities
	(cloud).width  = pf.getNumberOfParticles();
	(cloud).height = 1;
	(cloud).points.resize ((cloud).width * (cloud).height);

    //rstd::string imageName("/home/jamie/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8.bmp"); // by default
    std::string imageName("/home/jamie/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8_rot.bmp"); // by default
    //std::string imageName("/home/jazen/Documents/Classes/2015_Fall/16-831_Stats_in_Robotics/HW/HW_4/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8.bmp"); // by default
	cv::Mat map_image=cv::imread(imageName,CV_LOAD_IMAGE_GRAYSCALE);

	//Check that you got the image
	if(!map_image.data )
	{
		throw 0;
	}


    std::cout << "about to visualize" << std::endl;

    // open robot log
    //ifstream robotLog("/home/jazen/Documents/Classes/2015_Fall/16-831_Stats_in_Robotics/HW/HW_4/data/log/ascii-robotdata3.log");
    //ifstream robotLog("/home/jazen/Documents/Classes/2015_Fall/16-831_Stats_in_Robotics/HW/HW_4/data/log/robotdata1.log");
    ifstream robotLog("/home/jamie/Desktop/hw4_robostats/robotdata1.log");
    //ifstream robotLog("/home/jamie/Desktop/hw4_robostats/ascii-robotdata4.log");

    string logLine;
    //float maxLaserRangeFromLog = 0;
    if (robotLog.is_open()) {
        cout << "Opened log" << endl;
        while (getline(robotLog, logLine)) {

            boost::trim(logLine);
            std::vector<std::string> logLineSplit;
            boost::split(logLineSplit,logLine,boost::is_any_of(" "),boost::token_compress_on);

            if (logLine.at(0) == 'L') {
                //std::cout << "LASER DATA" << std::endl;
                std::vector<float> laserRanges;
                std::vector<float> laserWRTMap;
                for (uint i = 4; i < 7; i++) {
                    laserWRTMap.push_back(std::atof(logLineSplit.at(i).c_str()));
                }
                for (uint i = 7; i < logLineSplit.size(); i++) {
                    laserRanges.push_back(std::atof(logLineSplit.at(i).c_str()));
                }
                pf.laser(laserRanges, laserWRTMap);
            }
            else if (logLine.at(0) == 'O') {
                //std::cout << "ODOMETRY" << std::endl;
                std::vector<float> odometry;
                for (uint i = 1; i < logLineSplit.size(); i++) {
                    odometry.push_back(std::atof(logLineSplit.at(i).c_str()));
                }
                pf.odometry(odometry);
            }
            if (pf.getStepsUntilResample() == 0) {
                //pf.resample();
                pf.parallelResample();
            }

            // update cloud to share same number of particles
            (cloud).width  = pf.getNumberOfParticles();
            (cloud).height = 1;
            (cloud).points.resize ((cloud).width * (cloud).height);

            updateVisualization(pf, cloud, output,nh);
            //std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

    }
    else {
        cout << "Could not open log" << endl;
    }

    robotLog.close();

    //std::cout << "max laser range from log = " << maxLaserRangeFromLog << std::endl;

    std::cout << " ********** FINISHED ***********" << std::endl;






	return 0;
}

void updateVisualization(ParticleFilter &pf,
		pcl::PointCloud<pcl::PointXYZ> &cloud,
		sensor_msgs::PointCloud2 &output,
		ros::NodeHandle nh
)
{
    auto p = pf.getParticles();

//#pragma omp parallel for schedule(static,1) num_threads(8)
	for (int i=0; i<pf.getNumberOfParticles(); i++)
	{
        (cloud).points[i].x = p.at(i)->getX();
        (cloud).points[i].y = p.at(i)->getY();
        (cloud).points[i].z = p.at(i)->getWeight()*5;
	}

    //std::cout << "put everything in a cloud" << std::endl;

	//Convert between point cloud types
	pcl::toROSMsg(cloud, output);

	//Set the reference frame for your pointcloud (ROS bookkeeping)
	output.header.frame_id = std::string("/odom");

		output.header.stamp = ros::Time::now();
		pub.publish (output);

		ros::spinOnce ();

}


