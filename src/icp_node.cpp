#include <ros/ros.h>
#include <icp_node.h>

ScanMatchICP::ScanMatchICP(): lidar_scan_ptr(new sensor_msgs::PointCloud2), map_ptr(new sensor_msgs::PointCloud2), submap_ptr(new sensor_msgs::PointCloud2), icp_output_ptr(new sensor_msgs::PointCloud2){
	ROS_INFO_STREAM("\033[1;32m---> Scan Match with ICP started.\033[0m");
	
	gt_subscriber = nh.subscribe("gt",1,&ScanMatchICP::GtCallback,this);
	laser_scan_subscriber = nh.subscribe("scan",1,&ScanMatchICP::ScanCallback,this);

	map_publisher = nh.advertise<sensor_msgs::PointCloud2>("pc_map", 1);
	submap_pub = nh.advertise<sensor_msgs::PointCloud2>("submap", 1);
	point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
	lidarmatch_publisher = nh.advertise<sensor_msgs::PointCloud2>("lidarmatch", 1);
	result_publisher = nh.advertise<nav_msgs::Odometry>("result", 1);	

	tfListener.setExtrapolationLimit(ros::Duration(0.1));

	// tf::StampedTransform transform;
    // try {
    // 	listener.waitForTransform("base_link", "laser", ros::Time(0), ros::Duration(5.0));
    // 	listener.lookupTransform("base_link", "laser", ros::Time(0), transform); //tf: tranfer scan frame to car frame
    // 	pcl_ros::transformPointCloud(lidar_scan, lidar_car, transform);
    // 	std::cout << "transform"  << std::endl;
    // }
 	// catch (tf::TransformException ex) {
 	// 	ROS_ERROR("%s",ex.what());
	// 	ros::Duration(1.0).sleep();
	// 	exit(0);
  	// }
	
	is_first_scan = true;
	
	current_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
	submap_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
	map_pointcloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());	
	
	// lidar_scan_ptr = new sensor_msgs::PointCloud2;
}

ScanMatchICP::~ScanMatchICP(){
    ROS_INFO("Destroying ScanMatchICP");
}

void ScanMatchICP::GtCallback(const nav_msgs::Odometry::ConstPtr &msg){
	gtx = msg->pose.pose.position.x;
	gty = msg->pose.pose.position.y;
	gtz = msg->pose.pose.position.z;
	//initial guess
	if(is_first_scan){
		double yaw = 0;
    	tfMatrix << cos(yaw), -sin(yaw), 0.0,    0,
       	    	    sin(yaw),  cos(yaw), 0.0,    0,
                	     0.0,       0.0, 1.0,    0,
       	            	 0.0,       0.0, 0.0,  1.0;
	}
}
void ScanMatchICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
	std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
	
	if(is_first_scan){
		ConvertScan2PointCloud(scan_msg);
		is_first_scan = false;
		return;
	}
	// else{
	// 	*last_pointcloud = *current_pointcloud;
	// }
	
	ConvertScan2PointCloud(scan_msg);
	
	std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time-start_time);
	std::cout<<"\nconvert data time: " << time_used.count() << "s" << std::endl;
	
	start_time = std::chrono::steady_clock::now();
	
	ScanMatchWithICP(scan_msg);
	
	end_time = std::chrono::steady_clock::now();
	time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time-start_time);
	std::cout << "ICP time: " << time_used.count() << "s" << std::endl;

	MergePCDMap();
	std::cout << "Merge map" << std::endl; 
}


void ScanMatchICP::ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
    // cloud_msg->points.resize(scan_msg->ranges.size());

    // for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i){
    //     pcl::PointXYZ &point_tmp = cloud_msg->points[i];
    //     float range = scan_msg->ranges[i];
        
    //     if (!std::isfinite(range))
    //         continue;
            
    //     if (range > scan_msg->range_min && range < scan_msg->range_max){
    //         float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    //         point_tmp.x = range * cos(angle);
    //         point_tmp.y = range * sin(angle);
    //         point_tmp.z = 0.0;
    //     }
    // }

    // cloud_msg->width = scan_msg->ranges.size();
    // cloud_msg->height = 1;
    // cloud_msg->is_dense = true; // not contains nans
	projector.transformLaserScanToPointCloud("base_link", *scan_msg, *lidar_scan_ptr, tfListener);
    point_cloud_publisher.publish(lidar_scan_ptr);

    // pcl_conversions::toPCL(lidar_cloud_car, cloud_msg->header);
    
	if(is_first_scan){
		pcl::fromROSMsg(*lidar_scan_ptr, *map_pointcloud);
		// pcl::toROSMsg(*map_pointcloud, *map_ptr);
		// map_ptr->header.frame_id = "map";
		// map_ptr->header.stamp = ros::Time::now();
		// map_publisher.publish(*map_ptr);
		// *map_pointcloud = lidar_cloud_car;
		// lidar_map = lidar_cloud_car;
		// pcl::toROSMsg(lidar_map, map);
		// pcl::toROSMsg(lidar_cloud_car, map);
		// map = *lidar_scan_ptr;
	}
	else{
		// *lidar_scan_ptr = lidar_cloud_car;
		// *current_pointcloud = lidar_cloud_car;

	}
	// pcl::fromROSMsg(*lidar_scan_ptr, *map_pointcloud);

}

void ScanMatchICP::ScanMatchWithICP(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);
    
	pcl::fromROSMsg(*lidar_scan_ptr, *current_pointcloud);


	icp.setInputSource(current_pointcloud);
    icp.setInputTarget(map_pointcloud);
    
    icp.setMaxCorrespondenceDistance (1); // 5cm, correspondences with higher distances will be ignored
	icp.setMaximumIterations (50);
	icp.setTransformationEpsilon (1e-8); // angle
	icp.setEuclideanFitnessEpsilon (1); // distance


    icp.align(*Final, tfMatrix);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
    if (icp.hasConverged() == false){
        std::cout << "not Converged" << std::endl;
        return;
    }
    else{
        tfMatrix = icp.getFinalTransformation();
		std::cout << current_pointcloud->points.at(0).x << " " <<current_pointcloud->points.at(0).y << std::endl;
		std::cout << map_pointcloud->points.at(0).x << " " <<map_pointcloud->points.at(0).y << std::endl;
		std::cout << Final->points.at(0).x << " " << Final->points.at(0).y << std::endl;
        // Publish the result of lidar scan matching point cloud
		pcl::toROSMsg(*Final, *icp_output_ptr);
		icp_output_ptr->header.frame_id = "map";
		icp_output_ptr->header.stamp = ros::Time::now();
	    lidarmatch_publisher.publish(*icp_output_ptr);
        
//        float x, y, z, roll, pitch, yaw;
//        pcl::getTranslationAndEulerAngles(tfMatrix, x, y, z, roll, pitch, yaw);
//        std::cout << "transfrom: (" << x << ", " << y << ", " << yaw * 180 / M_PI << ")" << std::endl;
    	
		// Publish localization result 
		tf::Matrix3x3 rMatrix;   // get rotation matrix
		tf::Quaternion rtQ;
		geometry_msgs::Quaternion msgQ;

	    rMatrix.setValue(static_cast<double>(tfMatrix(0,0)), static_cast<double>(tfMatrix(0,1)), static_cast<double>(tfMatrix(0,2)), 
	               		 static_cast<double>(tfMatrix(1,0)), static_cast<double>(tfMatrix(1,1)), static_cast<double>(tfMatrix(1,2)), 
	               	  	 static_cast<double>(tfMatrix(2,0)), static_cast<double>(tfMatrix(2,1)), static_cast<double>(tfMatrix(2,2)));
	    	
		rMatrix.getRotation(rtQ);  // Quaternion representing of rotation
		quaternionTFToMsg(rtQ, msgQ);

		result.header.frame_id = "map";
		result.child_frame_id = "base_link";
		result.pose.pose.position.x = tfMatrix(0, 3);
		result.pose.pose.position.y = tfMatrix(1, 3);
		result.pose.pose.position.z = tfMatrix(2, 3);
		result.pose.pose.orientation = msgQ;
		result_publisher.publish(result);
	}
}

void ScanMatchICP::MergePCDMap(){
	// tf::StampedTransform transform;
	// tf::TransformListener tf;
    // listener.waitForTransform("map", "laser", ros::Time(0), ros::Duration(5.0));
    // listener.lookupTransform("map", "base_link", ros::Time(0), transform); //tf: tranfer scan frame to map frame
    // tf.setExtrapolationLimit(ros::Duration(0.1));
	// pcl_ros::transformPointCloud("map",*lidar_scan_ptr, submap, tf);
    // std::cout << "transform"  << std::endl;

	// std::cout << "transform: \n" << tfMatrix << std::endl;
	pcl_ros::transformPointCloud(tfMatrix, *icp_output_ptr, *submap_ptr);
	submap_ptr->header.frame_id = "map";
	submap_ptr->header.stamp = ros::Time::now();
	submap_pub.publish(*submap_ptr);

	// pcl::PointCloud<pcl::PointXYZ> transformed_map_cloud;
	// pcl::transformPointCloud(*map_pointcloud, transformed_map_cloud, tfMatrix);
	pcl::fromROSMsg(*submap_ptr, *submap_pointcloud);
	// pcl::fromROSMsg(*icp_output_ptr, *submap_pointcloud);
	*map_pointcloud += *submap_pointcloud;
	// *map_pointcloud += transformed_submap_cloud;
	// map_pointcloud->insert(map_pointcloud->begin(), submap_pointcloud->begin(), submap_pointcloud->end());
	// pcl::PCLPointCloud2::concatenate(*map_pointcloud,*submap_pointcloud);

	pcl::toROSMsg(*map_pointcloud, *map_ptr);
	map_ptr->header.frame_id = "map";
	map_ptr->header.stamp = ros::Time::now();
	map_publisher.publish(*map_ptr);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"icp_node");
    ScanMatchICP scan_match_icp;

    ros::spin();
    return 0;
}



