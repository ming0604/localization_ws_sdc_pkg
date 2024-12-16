// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
// #include "tf/transform_listener.h"
#include "message_filters/subscriber.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_listener");
    ros::NodeHandle nh;

    ros::Publisher listen_pose_pub_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // std::shared_ptr<tf2_ros::TransformListener> tfListener;
    // std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    geometry_msgs::TransformStamped transformStamped;

    // tf::TransformListener listener;
    // tf::StampedTransform transformStamped;
    
    // tfListener.setUsingDedicatedThread(true);
    // listen_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("listen_pose", 10, true);
    // listen_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("tracked_pose", 10, true);

    // tfBuffer.reset(new tf2_ros::Buffer());
    // tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));
    // tfBuffer.setUsingDedicatedThread(true);
    // tfBuffer.setTransformExpirationDuration(ros::Duration(60.0));
    // ros::Rate rate(100);
    ROS_WARN("OUT");

    while(nh.ok()){
        try{
            ROS_WARN("IN");
            ros::Time now = ros::Time(0);
            // ros::Time now = ros::Time::now();
            // ros::Duration(0.1).sleep();
            // tfBuffer.canTransform("base_link","map",ros::Duration(5.0));
            // transformStamped = tfBuffer.lookupTransform("base_link","map",now);
            // transformStamped = tfBuffer->lookupTransform("base_link","map",ros::Time(0));
            // listener.waitForTransform("/map", "/base_link", now, ros::Duration(3.0));
            // listener.lookupTransform("/map", "/base_link", now, transformStamped);
            // tfBuffer.waitForTransform("base_link", "map", now, ros::Duration(1.0), nullptr);
            // tfListener.lookupTransform("base_link", "map", ros::Time(0), transformStamped);
            // transformStamped = tfBuffer.lookupTransform("base_link", "map",now);
            transformStamped = tfBuffer.lookupTransform("base_link", "laser", now, ros::Duration(1.0));
            ROS_WARN("%f, %f\n",transformStamped.transform.translation.x,transformStamped.transform.translation.y);
        }
        catch(tf2::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            // ros::Duration(0.05).sleep();
        }
        // rate.sleep();

        ROS_INFO(" amcl_pose: %.6f %.6f",transformStamped.transform.translation.x,transformStamped.transform.translation.y);
        // ROS_INFO(" amcl_pose: %.6f %.6f",transformStamped.getOrigin().x(),transformStamped.getOrigin().y());

        geometry_msgs::PoseWithCovarianceStamped ffp;

        ffp.header.frame_id = "map";
        ffp.header.stamp = transformStamped.header.stamp; 
        ffp.pose.pose.position.x = transformStamped.transform.translation.x;
        ffp.pose.pose.position.y = transformStamped.transform.translation.y;
        ffp.pose.pose.orientation = transformStamped.transform.rotation;

        // ffp.header.stamp = transformStamped.header.stamp; 
        // ffp.pose.pose.position.x = transformStamped.getOrigin().x();
        // ffp.pose.pose.position.y = transformStamped.getOrigin().y();
        // ffp.pose.pose.orientation = transformStamped.getRotation();

        listen_pose_pub_.publish(ffp);
    }
    ros::spin();
}