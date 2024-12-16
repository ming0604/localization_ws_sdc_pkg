#ifndef ICP_NODE
#define ICP_NODE

#include <iostream>
#include <math.h>
#include <chrono>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class ScanMatchICP{
  private:
  	ros::NodeHandle nh;
  	ros::NodeHandle private_node;
	ros::Subscriber gt_subscriber;
  	ros::Subscriber laser_scan_subscriber;
	ros::Publisher point_cloud_publisher;
  	ros::Publisher map_publisher, submap_pub;
	ros::Publisher lidarmatch_publisher;
  	ros::Publisher result_publisher;
  	
  	sensor_msgs::PointCloud2::Ptr icp_output_ptr;
	sensor_msgs::PointCloud2::Ptr map_ptr;
	sensor_msgs::PointCloud2::Ptr submap_ptr;
	sensor_msgs::PointCloud2::Ptr lidar_scan_ptr;
  	nav_msgs::Odometry result;

	tf::TransformListener listener;
	tf::TransformListener tfListener;
	laser_geometry::LaserProjection projector;
	
  	Eigen::Matrix4f tfMatrix;
	double gtx, gty, gtz;
  	
  	bool is_first_scan;
  	
  	pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr submap_pointcloud;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr map_pointcloud;
	
	pcl::PointCloud<pcl::PointXYZ> lidar_map; // lidar points in map frame (world)
	pcl::PointCloud<pcl::PointXYZ> lidar_scan; //lidar points in scan frame (velodyne)
	pcl::PointCloud<pcl::PointXYZ> lidar_car; // lidar points in car frame (base_link)
  	
  	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  	
  	void ScanMatchWithICP(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  	void ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
	void MergePCDMap();

  public:
  	ScanMatchICP();
  	~ScanMatchICP();
  	void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);  	
	void GtCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif
