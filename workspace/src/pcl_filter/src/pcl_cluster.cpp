//The goal of this node is to take the full sensor_msgs::PointCloud2 created by the laser scans and filter it to remove the map PURELY based on geometry. This is not meant to do scan-matching and subtraction from the map.

//it reads from the my_cloud topic which is a point cloud in sensor_msgs::PointCloud2 form, then puts it into a pcl data structure so it can be filtered. When you create the filter though, it segfaults, even when you follow

//After the filtering it will transform the pcl structure back to a sensor_msgs::PointCloud2 for publishing


#include <ros/ros.h>
#include <visualization_msgs/Marker.h> 
#include <visualization_msgs/MarkerArray.h> 
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
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

//#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>



ros::Publisher pub_cluster1;
ros::Publisher pub_cluster2;
ros::Publisher pub_cluster3;
ros::Publisher pub_cluster4;
ros::Publisher pub_cluster5;
ros::Publisher pub_cluster6;

ros::Publisher marker_pub; 

ros::Publisher cc_pos;// clusterCenter positions

void publish_cloud(ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster){
  sensor_msgs::PointCloud2::Ptr clustermsg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*cluster , *clustermsg);
  clustermsg->header.frame_id = "/map";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish (*clustermsg);

}




/*
typedef Eigen::Array<size_t, 4, 1> Array4size_t;
void getMinMax3D (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                  Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
  // @todo fix this
  if (cloud->fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
      cloud->fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
      cloud->fields[z_idx].datatype != pcl::PCLPointField::FLOAT32)
  {
    PCL_ERROR ("[pcl::getMinMax3D] XYZ dimensions are not float type!\n");
    return;
  }

  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  size_t nr_points = cloud->width * cloud->height;

  Eigen::Array4f pt = Eigen::Array4f::Zero ();
  Array4size_t xyz_offset (cloud->fields[x_idx].offset, cloud->fields[y_idx].offset, cloud->fields[z_idx].offset, 0);

  for (size_t cp = 0; cp < nr_points; ++cp)
  {
    // Unoptimized memcpys: assume fields x, y, z are in random order
    memcpy (&pt[0], &cloud->data[xyz_offset[0]], sizeof (float));
    memcpy (&pt[1], &cloud->data[xyz_offset[1]], sizeof (float));
    memcpy (&pt[2], &cloud->data[xyz_offset[2]], sizeof (float));
    // Check if the point is invalid
    if (!pcl_isfinite (pt[0]) || 
        !pcl_isfinite (pt[1]) || 
        !pcl_isfinite (pt[2]))
    {
      xyz_offset += cloud->point_step;
      continue;
    }
    xyz_offset += cloud->point_step;
    min_p = (min_p.min) (pt);
    max_p = (max_p.max) (pt);
  }
  min_pt = min_p;
  max_pt = max_p;
}
*/
/*
// Function to create markers for each of the clusters we get
visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b)
{
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  int x_idx;int y_idx;int z_idx;
 
  pcl::compute3DCentroid (*cloud_cluster, centroid);
  getMinMax3D ((cloud_cluster),x_idx,y_idx,z_idx, min, max);
 
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = cloud_cluster->header.frame_id;
  marker.header.stamp = ros::Time::now();
 
  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
 
  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
 
  marker.scale.x = (max[0]-min[0]);
  marker.scale.y = (max[1]-min[1]);
  marker.scale.z = (max[2]-min[2]);
 
  if (marker.scale.x ==0)
      marker.scale.x=0.1;

  if (marker.scale.y ==0)
    marker.scale.y=0.1;

  if (marker.scale.z ==0)
    marker.scale.z=0.1;
   
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();
//   marker.lifetime = ros::Duration(0.5);
  return marker;
}
*/
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
      empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
      cluster_vec.push_back(empty_cluster);
    }

/*
    // POP: Computing features from the clustered point clouds:
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cluster_vec[0]);// For now testing on cluster_1 alone
    feature_extractor.compute ();
    // Below are the variables to store the feature descriptors and the bounding box. Add/remove as necessary
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

   
    // Get the computed features
    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    // Calculate the position and orientation of the clusters
     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
     Eigen::Quaternionf quat (rotational_matrix_OBB);

     // Convert Eigne::Vector3f to pcl:PointXYZ
    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    p1 = rotational_matrix_OBB * p1 + position;
    p2 = rotational_matrix_OBB * p2 + position;
    p3 = rotational_matrix_OBB * p3 + position;
    p4 = rotational_matrix_OBB * p4 + position;
    p5 = rotational_matrix_OBB * p5 + position;
    p6 = rotational_matrix_OBB * p6 + position;
    p7 = rotational_matrix_OBB * p7 + position;
    p8 = rotational_matrix_OBB * p8 + position;

    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));
    

*/
    // POP: End of feature extraction
/*


////// POP: ClusterCenter pos for tracking
*/
// Publishing as a Float32MultiArray coz vector<geometry_msgs::Point>
// cant be published without a custom msg types.    

std_msgs::Float32MultiArray cc;
for(int i=0;i<6;i++)
{
  cc.data.push_back(cluster_vec[i]->points[cluster_vec[i]->points.size()/2].x);
  cc.data.push_back(cluster_vec[i]->points[cluster_vec[i]->points.size()/2].y);
  cc.data.push_back(cluster_vec[i]->points[cluster_vec[i]->points.size()/2].z);

}

cc_pos.publish(cc);

////// POP: EO Clustercenter pos for tracking

//// POP: cluster viz end



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
  ros::init (argc, argv, "my_pcl_cluster");
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


  cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1



  // Spin
  ros::spin ();
}

