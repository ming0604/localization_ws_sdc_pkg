#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <sstream>
#include <sys/shm.h>
#include <sys/sem.h>

// roscpp
#include <ros/ros.h>

// Messages that I need
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include "parameters.h"
#include "signalProcessing.h"

void          *shmPtr_ros = NULL;
int           shmid_ros;
int           semid_ros;
key_t         shmKey_ros = (key_t)123;
key_t         semKey_ros = (key_t)456;

struct sembuf sop; // for P() V() operation
long          page_size = sysconf(_SC_PAGESIZE);

AMCLPOSE 	*amclPtr, amcl;

int P(int s);
int V(int s);
void amclReceived(const nav_msgs::OdometryConstPtr& msg);

int main(int argc, char **argv){
	
	ros::init(argc, argv, "shm_ros");
	ROS_INFO("shm_ros");

	ros::NodeHandle n;
	ros::Subscriber amcl_sub;

	// ======================== shm init ========================
	if((shmid_ros = shmget(shmKey_ros, page_size, IPC_CREAT | 0666)) == -1){
		printf("shmget error\n");
		exit(1);
	}
	if((shmPtr_ros = shmat(shmid_ros, NULL, 0)) == (void*)-1){
		printf("shmat error\n");
		exit(1);
	}

	// ======================== sem init ========================
	if((semid_ros = semget(semKey_ros, 1, IPC_CREAT | 0666)) == -1){
		printf("semget error\n");
		exit(1);
	}

	amcl_sub = n.subscribe("/amcl_ekf", 100, &amclReceived);

	ros::spin();

	amclPtr = &amcl;
	amclPtr = (AMCLPOSE*)shmPtr_ros;

	ros::Rate loop_rate(0.01);

	while(ros::ok()){
		if(amclPtr->valid){
			P(semid_ros);
			// amclPtr =shmPtr_ros;
			printf("writing in shm\n");
			V(semid_ros);
			loop_rate.sleep();
		}else{
			loop_rate.sleep();
		}
	}
	return 0;
}

void amclReceived(const nav_msgs::OdometryConstPtr& msg){
	ROS_INFO("amclReceived");
	// ROS_INFO("amcl_t: %ld", msg->header.stamp.toNSec());
	// amclPtr->time = ros::Time::now().toNSec();
	// printf("amcl_t: %lf\n", (amcl.time - 1.6769641e+18));
	amclPtr->pose[0] = msg->pose.pose.position.x;
	amclPtr->pose[1] = msg->pose.pose.position.y;
	amclPtr->pose[2] = tf2::getYaw(msg->pose.pose.orientation);

	amclPtr->linVel[0] = msg->twist.twist.linear.x;
	amclPtr->linVel[1] = msg->twist.twist.linear.y;
	amclPtr->angVel[2] = msg->twist.twist.angular.z;

	amclPtr->valid = true;
}

int P(int s){ // aquire (try to decrease)
	sop.sem_num = 0;
	sop.sem_op = -1;
	sop.sem_flg = 0;

	if(semop(s, &sop, 1) < 0){
		printf("semop error: (P(sem))\n");
		return -1;
	}
	return 0;
}

int V(int s){ // release (increase)
	sop.sem_num = 0;
	sop.sem_op = 1;
	sop.sem_flg = 0;

	if(semop(s, &sop, 1) < 0){
		printf("semop error: (V(sem))\n");
		return -1;
	}
	return 0;
}