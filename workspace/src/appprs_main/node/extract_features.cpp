#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../circleFit/CircleFitByTaubin.cpp"
#include <time.h>
#include <iostream>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/boundary.h>

void cloud1_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	//http://answers.ros.org/question/32966/euclidean-cluster-extraction/

	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input, *input_cloud);
	int Size = (*input_cloud).size();

	reals X[Size];
	reals Y[Size];

	// std::vector<pcl::PointXYZ> all_pts=(*input_cloud).points;

	pcl::PointCloud<pcl::PointXYZ> internal_cloud = (*input_cloud);
	float meanX = 0;
	float meanY = 0;
	std::vector<int>::const_iterator pit;

	//Extract Points from Cloud for fast lookup
	int j = 0;
	for (int i = 0; i < Size; i++) {
		X[i] = internal_cloud.points[i].x;
		Y[i] = internal_cloud.points[i].y;

	}


	//Calculating Circles and stuff
	Data data1(Size, X, Y);
	Circle circle;
	cout.precision(7);
	circle = CircleFitByTaubin(data1);
	cout << "\n  Taubin fit:  center (" << circle.a << "," << circle.b
			<< ")  radius " << circle.r << "  sigma " << circle.s << std::endl;

	//Calculate Mean
	meanX = data1.meanX;
	meanY = data1.meanY;

	//Calculating STD of Points away from mean
	internal_cloud = (*input_cloud);
	float std_from_mean = 0;
	for (int i = 0; i < Size; i++) {
		std_from_mean += pow(X[i] - meanX, 2) + pow(Y[i] - meanY, 2);
	}
	float temp_err = std_from_mean;
	std_from_mean = sqrt(1 / Size * temp_err);

	int features[13];

	features[0] = Size; //number of points
	features[1] = std_from_mean;  //Standard Dev from Mean
	features[2] = 0; //mean distance from median
	features[3] = 0; //jump distance from last segment (not doing)
	features[4] = 0; //just distance from next segment (nto doing)
	features[5] = sqrt(pow(X[Size - 1] - X[0], 2) + pow(Y[Size - 1] - Y[0], 2)); //width
	features[6] = 0;
	features[7] = 0;
	features[8] = circle.s; //Sum of squared residuals
	features[9] = circle.r;
	features[10] = 0;
	features[11] = 0;
	features[12] = 0;

}

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "my_pcl_filter");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("cluster_1", 1, cloud1_cb);

	// Spin
	ros::spin();
}
