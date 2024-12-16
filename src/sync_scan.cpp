#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace message_filters;

class Node{
 public:
  Node(){
	// ROS_INFO("initial");
    laser_sub_1.subscribe(nh, "/front_scan", 1);
    laser_sub_2.subscribe(nh, "/back_scan", 1);

    scan_pub = nh.advertise<sensor_msgs::LaserScan> ("/scan", 5);
    
    sync_.reset(new Sync(MySyncPolicy(10), laser_sub_1, laser_sub_2));

    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2));
  }

  void callback(const sensor_msgs::LaserScanConstPtr& scan_1, const sensor_msgs::LaserScanConstPtr& scan_2){
	// ROS_INFO("callback");
  
  scan.header.stamp = scan_1->header.stamp;
	scan.header.frame_id = "laser";
	scan.angle_min = -3.1415926;
	scan.angle_max = 3.1415926;
	scan.angle_increment = scan_1->angle_increment;
	scan.time_increment = scan_1->time_increment;
	scan.scan_time = scan_1->scan_time;
	scan.range_min = scan_1->range_min;
	scan.range_max = scan_1->range_max;

  scan.ranges.resize(num_readings);
	for(i = 0; i <= 360; i++)
		scan.ranges[i] = scan_2->ranges[360-i];
	for(j = 361; j <= 720; j++)
		scan.ranges[j] = scan_1->ranges[1080-j];
	for(k = 721; k <= 1080; k++)
		scan.ranges[k] = scan_1->ranges[1080-k];
	for(l = 1081; l <= 1440; l++)
		scan.ranges[l] = scan_2->ranges[1800-l];

	scan_pub.publish(scan);
  }

 private:
  ros::NodeHandle nh;
  tf::TransformListener listener;
  
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_1;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_2;

  ros::Publisher scan_pub;
  sensor_msgs::LaserScan scan;

  int i = 0, j = 0, k = 0, l = 0;
  int num_readings = 1440;
  // double ranges[num_readings];

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_scan");

  Node sync_scan;

  ros::spin();
}