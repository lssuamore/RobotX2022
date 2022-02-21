#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "iostream"
#include "stdio.h"
#include "time.h"
//#include <obj_recognition/SegmentedClustersArray.h>
//#include <obj_recognition/ClusterData.h>

//sensor_msgs::PointCloud out_pointcloud;
//sensor_msgs::convertPointCloud2ToPointCloud


double latitude;
double longitude;
double th;

ros::Publisher lidar_pub;

/* 	float centriodx;
	float centriody; */
 
 
 void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	th = gps_msg->altitude; //sets altitude rom gps
	/* printf("the latitude is: %f\n", latitude); //prints
	printf("the longitude is: %f\n", longitude);
	printf("the altitude is: %f\n", th); */
}
 
 

void LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
	
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; // pointer to empty pointcloud2 struct cloud
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_p  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f  (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr downsampled (new sensor_msgs::PointCloud2);
	
	pcl_conversions::toPCL(*lidar_msg, *cloud);
	printf ("Before filtering Cloud= %d Points.\n", cloud->width * cloud->height); //has 126 points
	
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //VoxelGrid downsamples the lidar data
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.08f, 0.08f, 0.08f); //2 cm leaf size
	sor.filter (*cloud);
	
	//printf ("After filtering Cloud using VoxelGrid = %d Points.\n", cloud->width * cloud->height); //has 126 points
	  std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
	
	pcl::fromPCLPointCloud2(*cloud, *downsampled_XYZ);
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ_filtered (downsampled_XYZ);
	
	pcl::PassThrough<pcl::PointXYZ> pass;
/* 	pass.setInputCloud (downsampled_XYZ);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 2.0);
	pass.filter (*downsampled_XYZ); */
	
	pass.setInputCloud (downsampled_XYZ);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (0, 30.0);
	pass.filter (*downsampled_XYZ);
	
	pass.setInputCloud (downsampled_XYZ);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-30.0, 30.0);
	pass.filter (*downsampled_XYZ);
	
	  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *downsampled_XYZ)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
	
	
	pcl::PCDWriter writer;
	
	/* seg.setOptimizeCoefficients (true);
	
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	
	seg.setDistanceThreshold (0.01);
	 */
	
	
/* 	int nr_points = (int) downsampled_XYZ->points.size ();
	//printf ("After filtering Cloud using segmentation = %d Points.\n",nr_points); //has 126 points

    //Contains the plane point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    // While 30% of the original cloud is still there
    while (downsampled_XYZ->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (downsampled_XYZ);
        seg.segment (*inliers, *coefficients);
		//printf ("After filtering Cloud using segmentation = %ld Points.\n", downsampled_XYZ->points.size()); //has 126 points

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

	printf ("indicies = %ld Points.\n", inliers->indices.size ());

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (downsampled_XYZ);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        downsampled_XYZ.swap(cloud_f);
    } */
   
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (downsampled_XYZ);
	
	printf ("After filtering Cloud using tree = %ld Points.\n", downsampled_XYZ->points.size()); //has 126 points

	std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.8); // 2cm
    ec.setMinClusterSize (0);
    ec.setMaxClusterSize (2000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (downsampled_XYZ);
    ec.extract (cluster_indices);
	
	printf ("indicies size = %ld Points.\n", cluster_indices.size()); //prints zero
	printf ("indicies length = %ld Points.\n", (cluster_indices.end() - cluster_indices.begin())); //prints zero
	
/* 	float centriodx[cluster_indices.size()];
	float centriody[cluster_indices.size()]; */
	float centriodx;
	float centriody;
	
	int j = 0;
	//never enters the for loop
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
     for (const auto& idx : it->indices)
     cloud_cluster->push_back ((*downsampled_XYZ)[idx]); 
     cloud_cluster->width = cloud_cluster->size ();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;
     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
     std::stringstream ss;
     ss << "cloud_cluster_" << j << ".pcd";
     writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
	/*  pcl::PointXYZRGB centroid;
	std::vector<pcl::PointXYZRGB> centroids;
	pcl::computeCentroid( *(cloud_cluster), centroid);
      centroids.push_back(centroid);
      std::cout<<"center of buoy #"<<j+1<<" in ("<<(centroids[j]).x<<", "<<(centroids[j]).y<<", "<<(centroids[j]).z<<")"<<std::endl; */
	  for (const auto& point: *cloud_cluster) {
	  centriodx = point.x + centriodx; 
	  centriody = point.y + centriody; 
	  }
	  centriodx = centriodx / cloud_cluster->size(); 
	  centriody = centriody / cloud_cluster->size(); 
	  printf("x is %f\n", centriodx);
	  printf("y is %f\n", centriody);
	  geometry_msgs::Point lidar_point;
	  lidar_point.x = centriodx;
	lidar_point.y = centriody;
	lidar_point.z = cluster_indices.size();  //this will give me the the number of clusters in lidar_point.z
	lidar_pub.publish(lidar_point); 
	printf("%f\n", lidar_point.x);
	printf("%f\n", lidar_point.y);
	  centriodx = 0;
	  centriody = 0;
	  j++;
   }
	
/* 	  std::cerr << "Cloud cluster: " << std::endl;
  for (const auto& point: *cloud_cluster)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
	 */
	
	
/* 	pcl::PointXYZ centre;
	for (size_t i = 0; i < cluster_indices.size(); i++){
	pcl::computeCentroid(*cloud_cluster, centre);
	std::cout<<"center of pipe #" "("<<centre.x<<", "<<centre.y<<", "<<centre.z<<")"<<std::endl;
	 } */
	 
	/*  pcl::PointXYZRGB centroid;
	std::vector<pcl::PointXYZRGB> centroids;
     //for (size_t i = 0; i < cloud_cluster->size(); i++){
	for (size_t i = 0; i < cluster_indices.size(); i++){
		//for (size_t j = 0; j < cloud_cluster->size(); j++){
      pcl::computeCentroid( *(cloud_cluster), centroid);
      centroids.push_back(centroid);
      std::cout<<"center of buoy #"<<i+1<<" in ("<<(centroids[i]).x<<", "<<(centroids[i]).y<<", "<<(centroids[i]).z<<")"<<std::endl;
    } */
	//}
	
	
	
	
	
	
/* 	printf ("After filtering Cloud = %d Points.\n", segPtr->width * segPtr->height); //has 126 points
    printf ("After filtering Cloud = %d Points.\n", cloud->width * cloud->height); //has 648 points */

/* for (const auto& point: *cloud_point_ptr)
	{
		printf("%f\n", point.x); 
		printf("%f\n", point.y); 
		printf("%f\n", point.z);
	} 

for (const auto& point: *segPtr)
	{
		printf("segment %f\n", point.x); 
		printf("segment %f\n", point.y); 
		printf("segment %f\n", point.z);
	}  */


}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Lidar");

	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;

	ros::Subscriber topic_sub = nh.subscribe("/wamv/sensors/lidars/lidar_wamv/points", 100, LidarCallBack); //subscribes to Lidar
	
	ros::Subscriber topic_sub1 = nh1.subscribe("/wamv/sensors/gps/gps/fix", 10, gpsCallBack);
	
	lidar_pub = nh2.advertise<geometry_msgs::Point>("lidar_point", 100); 

	ros::Rate loop_rate(100);

	while(ros::ok()) {

	ros::spinOnce();
	loop_rate.sleep();
	
	}

}
