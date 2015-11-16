//The goal of this node is to take the full sensor_msgs::PointCloud2 created by the laser scans and filter it to remove the map PURELY based on geometry. This is not meant to do scan-matching and subtraction from the map.

//it reads from the my_cloud topic which is a point cloud in sensor_msgs::PointCloud2 form, then puts it into a pcl data structure so it can be filtered. When you create the filter though, it segfaults, even when you follow

//After the filtering it will transform the pcl structure back to a sensor_msgs::PointCloud2 for publishing


#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub_cluster1;
ros::Publisher pub_cluster2;
ros::Publisher pub_cluster3;
ros::Publisher pub_cluster4;
ros::Publisher pub_cluster5;
ros::Publisher pub_cluster6;

void publish_cloud(ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster){
  sensor_msgs::PointCloud2::Ptr clustermsg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*cluster , *clustermsg);
  clustermsg->header.frame_id = "/map";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish (*clustermsg);

}

// static_test_with_point_clouds.bag is publishing PointCloud on /my_cloud, not PointCloud2...
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //http://answers.ros.org/question/32966/euclidean-cluster-extraction/

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *input_cloud);

  tree->setInputCloud (input_cloud);

  /* Here we are creating a vector of PointIndices, which contains the actual index
   * information in a vector<int>. The indices of each detected cluster are saved here.
   * Cluster_indices is a vector containing one instance of PointIndices for each detected 
   * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
   */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.08); 
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (600);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);

  /* To separate each cluster out of the vector<PointIndices> we have to 
   * iterate through cluster_indices, create a new PointCloud for each 
   * entry and write all points of the current cluster in the PointCloud. 
   */
  //pcl::PointXYZ origin (0,0,0);
  //float mindist_this_cluster = 1000;
  //float dist_this_point = 1000;

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
          //push_back: add a point to the end of the existing vector
                  cloud_cluster->points.push_back(input_cloud->points[*pit]);
                  //dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
                  //                                          origin);
                  //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
          }

          //Merge current clusters to whole point cloud
      //*clustered_cloud = *cloud_cluster;
      //*clustered_cloud += *cloud_cluster;
      cluster_vec.push_back(cloud_cluster);

    }

    //Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < 6){
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      cluster_vec.push_back(empty_cluster);
    }

    publish_cloud(pub_cluster1, cluster_vec[0]);
    publish_cloud(pub_cluster2, cluster_vec[1]);
    publish_cloud(pub_cluster3, cluster_vec[2]);
    publish_cloud(pub_cluster4, cluster_vec[3]);
    publish_cloud(pub_cluster5, cluster_vec[4]);
    publish_cloud(pub_cluster6, cluster_vec[5]);
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("filtered_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_cluster1 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_1", 1);
  pub_cluster2 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_2", 1);
  pub_cluster3 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_3", 1);
  pub_cluster4 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_4", 1);
  pub_cluster5 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_5", 1);
  pub_cluster6 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_6", 1);

  // Spin
  ros::spin ();
}

