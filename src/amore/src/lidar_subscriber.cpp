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
#include "amore/NED_waypoints.h"
#include "iostream"
#include "stdio.h"
#include "time.h"

int quantity;
int keep_track;

struct Point {
	float x;
	float y;
};

//lidar array
float x_lidar[100], y_lidar[100];

//lidar publisher
ros::Publisher lidar_pub;
 
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
	printf ("Before filtering Cloud= %d Points.\n", cloud->width * cloud->height); 
	
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //VoxelGrid downsamples the lidar data
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.08f, 0.08f, 0.08f); //8 cm leaf size
	sor.filter (*cloud);
	
	std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height 
    << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
	
	pcl::fromPCLPointCloud2(*cloud, *downsampled_XYZ);
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	
	pcl::PassThrough<pcl::PointXYZ> pass;
	
	pass.setInputCloud (downsampled_XYZ); 
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (0, 60.0); //these will need to change depending on how the frame of camera is
	pass.filter (*downsampled_XYZ);
	
	pass.setInputCloud (downsampled_XYZ);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-30.0, 30.0); //this is left and right in the x direction
	pass.filter (*downsampled_XYZ);
	
	std::cerr << "Cloud after filtering: " << std::endl;
	for (const auto& point: *downsampled_XYZ)
		std::cerr << "    " << point.x << " "
		<< point.y << " "
		<< point.z << std::endl;
	
	
	pcl::PCDWriter writer;
   
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (downsampled_XYZ);
	
	printf ("After filtering Cloud using tree = %ld Points.\n", downsampled_XYZ->points.size()); //has 126 points

	std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.8); // 80cm
    ec.setMinClusterSize (0);
    ec.setMaxClusterSize (2000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (downsampled_XYZ);
    ec.extract (cluster_indices);
	
	printf ("indicies size = %ld Points.\n", cluster_indices.size()); //prints zero
	printf ("indicies length = %ld Points.\n", (cluster_indices.end() - cluster_indices.begin())); //prints zero
	
	float centriodx;
	float centriody;
	
	keep_track = 0;
	int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
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
		for (const auto& point: *cloud_cluster) {
			centriodx = point.x + centriodx; 
			centriody = point.y + centriody; 
		}
		quantity = cluster_indices.size();
		centriodx = centriodx / cloud_cluster->size(); 
		centriody = centriody / cloud_cluster->size(); 
		x_lidar[j] = centriodx;
		y_lidar[j] = centriody;
		printf("x is %f\n", centriodx);
		printf("y is %f\n", centriody);
		centriodx = 0;
		centriody = 0;
		if (j == (cluster_indices.size() -1)) {	  
			amore::NED_waypoints lidar_point;
			Point point_array[100];
			Point point;
			for (int i=0; i<100; i++) {
				point.x = i;
				point.y = i;
				point_array[i] = point;
			}
			std::vector<Point> point_vector (point_array, point_array + sizeof(point_array) / sizeof(Point));
			lidar_point.points.clear();
			lidar_point.quantity = cluster_indices.size();
			for (int i = 0; i < cluster_indices.size(); i++) {
				geometry_msgs::Point point;
				point.x = x_lidar[i];
				point.y = y_lidar[i];
				point.z = 0;
				lidar_point.points.push_back(point);
				printf("x in array is %f\n", point.x);
				printf("y in array is %f\n", point.y);
			}  
			lidar_pub.publish(lidar_point); 
			ROS_INFO("Points HAVE BEEN PUBLISHED");
		}
		j++;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Lidar");

	ros::NodeHandle nh;
	ros::NodeHandle nh2;

	ros::Subscriber topic_sub = nh.subscribe("/wamv/sensors/lidars/lidar_wamv/points", 100, LidarCallBack); //subscribes to Lidar
	
	lidar_pub = nh2.advertise<amore::NED_waypoints>("lidar_point", 100); 

	ros::Rate loop_rate(100);

	while(ros::ok()) {
	

	ros::spinOnce();
	loop_rate.sleep();
	
	}

}
