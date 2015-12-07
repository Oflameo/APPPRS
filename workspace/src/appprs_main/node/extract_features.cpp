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
#include <ros/time.h>
#include <iostream>
#include <iomanip>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/boundary.h>
#include <vector>
#include <stdlib.h>

std::ofstream myfile;

void get_features(const sensor_msgs::PointCloud2ConstPtr& input,
		const int cloud_number) {
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
	for (int i = 0; i < Size; i++) {
		X[i] = internal_cloud.points[i].x;
		Y[i] = internal_cloud.points[i].y;
	}

	//Calculating Circles and stuff
	Data data1(Size, X, Y);
	Circle circle;
	circle = CircleFitByTaubin(data1);
	//	cout << "\n  Taubin fit:  center (" << circle.a << "," << circle.b
	//			<< ")  radius " << circle.r << "  sigma " << circle.s << std::endl;

	//Calculate Mean
	meanX = data1.meanX;
	meanY = data1.meanY;

	//Calcualte distances between consecutive points
	float dist_mean = 0;
	float boundary_length = 0;
	float angle_avg;
	float dist[Size - 1];
	for (int j = 0; j < Size - 1; j++) {
		float X1 = X[j];
		float X2 = X[j + 1];
		float Y1 = Y[j];
		float Y2 = Y[j + 1];

		dist[j] = sqrt(pow(X2 - X1, 2) + pow(Y2 - Y1, 2));
		boundary_length += dist[j];

		float dot = X1 * X2 + Y1 * Y2;
		float det = X1 * Y2 - Y1 * X2;
		angle_avg += atan2(det, dot);

	}
	dist_mean = boundary_length / (Size - 1);
	angle_avg = angle_avg / (Size - 1);
	//Calculate Curvature between 3 pts
	float avg_curve = 0;
	for (int i = 1; i < Size - 2; i++) {
		float Ax = X[i];
		float Ay = Y[i];
		float Bx = X[i + 1];
		float By = Y[i + 1];
		float Cx = X[i + 2];
		float Cy = Y[i + 2];

		float A = std::abs(
				((Ax * (By - Cy) + Bx * (Cy - Ay) + Cx * (Ay - By)) / 2));
		float dA = sqrt(pow(Bx - Ax, 2) + pow(By - Ay, 2));
		float dB = sqrt(pow(Cx - Bx, 2) + pow(Cy - By, 2));
		float dC = sqrt(pow(Ax - Cx, 2) + pow(Ay - Cy, 2));

		avg_curve += 4 * A / (dA * dB * dC);
	}
	avg_curve = avg_curve / (Size - 2);

	//boundary standard Dev
	float std_dist = 0;
	for (int j = 0; j < Size - 1; j++) {
		std_dist += pow(dist[j] - dist_mean, 2);
	}
	std_dist = sqrt(1.0 / ((float) Size - 1) * std_dist);

	//Calculating STD of Points away from mean
	float std_xy = 0;
	for (int i = 0; i < Size; i++) {
		std_xy += pow(X[i] - meanX, 2) + pow(Y[i] - meanY, 2);
	}
	float temp_err = std_xy;
	std_xy = sqrt(1.0 / (float) Size * temp_err);

	int features[13];

	float SCALEME = 100000;
	features[0] = Size; //number of points
	features[1] = std_xy * SCALEME;  //Standard Dev from Mean
	features[2] = 0.5 * SCALEME; //mean distance from median
	features[3] = 1 * SCALEME; //jump distance from last segment (not doing)
	features[4] = 0.5 * SCALEME; //just distance from next segment (nto doing)
	features[5] = sqrt(pow(X[Size - 1] - X[0], 2) + pow(Y[Size - 1] - Y[0], 2))
			* SCALEME; //width
	features[6] = 1 * SCALEME;
	features[7] = circle.s * SCALEME; //Sum of squared residuals
	features[8] = circle.r * SCALEME; //radius of circle
	features[9] = boundary_length * SCALEME;
	features[10] = std_dist * SCALEME; //standard deviation of distances between points
	features[11] = avg_curve * SCALEME; //mean curvature of object
	features[12] = angle_avg * SCALEME; //mean angle between consecutive points

	myfile << ros::Time::now() << ',' << cloud_number << ',';
	for (int i = 0; i < 13; i++) {

		myfile << std::setprecision(5) << features[i] << ',';
	}
	myfile << std::endl;

//	myfile.close();

}

void cloud1_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	if (!(*input).data.empty()) {
		get_features(input, 1);
	}
}
void cloud2_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	if (!(*input).data.empty()) {
		get_features(input, 2);
	}
}
void cloud3_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	if (!(*input).data.empty()) {
		get_features(input, 3);
	}
}
void cloud4_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	if (!(*input).data.empty()) {
		get_features(input, 4);
	}
}
void cloud5_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	if (!(*input).data.empty()) {
		get_features(input, 5);
	}
}
void cloud6_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	if (!(*input).data.empty()) {
		get_features(input, 6);
	}
}

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "my_pcl_filter");
	ros::NodeHandle nh;

	myfile.open("features.csv");

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub1 = nh.subscribe("cluster_1", 1, cloud1_cb);
	ros::Subscriber sub2 = nh.subscribe("cluster_2", 1, cloud2_cb);
	ros::Subscriber sub3 = nh.subscribe("cluster_3", 1, cloud3_cb);
	ros::Subscriber sub4 = nh.subscribe("cluster_4", 1, cloud4_cb);
	ros::Subscriber sub5 = nh.subscribe("cluster_5", 1, cloud5_cb);
	ros::Subscriber sub6 = nh.subscribe("cluster_6", 1, cloud6_cb);

	// Spin
	ros::spin();
}
