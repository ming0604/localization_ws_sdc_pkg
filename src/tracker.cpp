#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "signalProcessing.h"

// =============== functions ===================
int P(int s);
int V(int s);

// ================ globals ====================
unsigned int  cnt=0;
unsigned int  lcnt=0;
void          *shmPtr = NULL;
int           shmid;
int           semid;
key_t         shmKey = (key_t)666;
key_t         semKey = (key_t)777;
struct sembuf sop; // for P() V() operation
long          page_size = sysconf(_SC_PAGESIZE);
char          *chPtr;
double        sec=0;
double 		  lsec=0;

// struct sigaction sa;
// struct itimerval timer;

TRACKER          tck, *tckPtr;
CG               cg, *cgPtr;

// ros::Timer timer;
// ros::Time current_time;
// ros::Publisher gt_pub;

// std::chrono::high_resolution_clock::time_point start;
// std::chrono::high_resolution_clock::time_point end;
// std::chrono::nanoseconds duration;

// void timerRoutine(int signum){
// 	start = std::chrono::high_resolution_clock::now();

// 	// voltage2pulseWidth(voltage, pulseWidth, direction);
// 	// outputPwmDir(pulseWidth, poChannel, direction, instantDoCtrl);

// 	// getTiresOmega(tiresPtr); // get tires' angular velocity

// 	// get tracker data from shm //
// 	P(semid);
// 	shm2tck(shmPtr, tckPtr);
// 	V(semid);
// 	if(!tck.valid){
// 		printf("[Lose connect with the tracker.]\n");
// 		lcnt++;
// 		lsec = lcnt*TS;
// 		// emergencyStop(&timer);
// 	}

// 	tck2cg(tck, cgPtr);      // calculate data of cg
// 	// cg2tires(cg, tiresPtr);  // calculate data of tires

// 	if (cnt%100 == 0){
// 		showData(tck);
// 		printf("sec: %d ", (int)sec);
// 		printf("lsec: %d ", (int)lsec);
// 		end = std::chrono::high_resolution_clock::now();
// 		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
// 		printf(",  current routine used %lf ms.\n\n", duration.count()/1000000.0);
// 	}
// 	cnt ++;
// 	sec = cnt*TS;
// }

class Tracker{
	public:
		Tracker(){
			vive_on = true;
			tru_vive = true;
			is_cali = false;
			cali_rotate.setIdentity();
			cali_trans.setZero();

			gt_pub = n.advertise<nav_msgs::Odometry>("gt", 10);
			b_gt_pub = n.advertise<nav_msgs::Odometry>("base_gt_odom", 10);

			timer = n.createTimer(ros::Duration(0.01), &Tracker::timerCallback, this);
		}

		void timerCallback(const ros::TimerEvent& e){
			vive_on = true;
			// get tracker data from shm //
			P(semid);
			shm2tck(shmPtr, tckPtr);
			V(semid);
			if(!tck.valid){
				printf("[Lose connect with the tracker.]\n");
				lcnt++;
				lsec = lcnt*TS;
				// emergencyStop(&timer);
			}

			tck2cg(tck, cgPtr);      // calculate data of cg
			// cg2tires(cg, tiresPtr);  // calculate data of tires

			current_time = ros::Time::now();

			// if(cnt >= 0){

			// 	// listener.waitForTransform("t265_link", "vive", ros::Time(0), ros::Duration(3.0));
			// 	// listener.lookupTransform("t265_link", "vive", ros::Time(0), tracker_t265);
			// 	// tf::Quaternion tf_quat;
			// 	// transform.getRotation(tf_quat);
			// 	// printf("%f %f %f %f\n", transform.getRotation().x(), transform.getRotation().y()
			// 	// 					  , transform.getRotation().z(), transform.getRotation().z());
				
			// 	tf::Vector3 vorigin;
			// 	tf::Vector3 origin;

			// 	vorigin.setValue(0.90455968, 0.03565007, 0.03044991);
			// 	origin.setValue(tck.pose[0][3], tck.pose[1][3], tck.pose[2][3]);

			// 	vive_t265.setOrigin(vorigin);
			// 	transform.setOrigin(origin);

			// 	// static_transform.transform.translation.x = tck.poseRaw[0][3];
			// 	// static_transform.transform.translation.y = tck.poseRaw[1][3];
			// 	// static_transform.transform.translation.z = tck.poseRaw[2][3];

			// 	tf::Quaternion vquat;
			// 	tf::Quaternion quat;
			// 	tf::Matrix3x3 rotate;

			// 	vquat.setValue(-0.01940629, -0.01151683, 0.0128923, 0.99966308);
			// 	rotate.setValue(tck.pose[0][0],tck.pose[0][1],tck.pose[0][2],
			// 					tck.pose[1][0],tck.pose[1][1],tck.pose[1][2],
			// 					tck.pose[2][0],tck.pose[2][1],tck.pose[2][2]);
			// 	// rotate.inverse();

			// 	rotate.getRotation(quat);
			// 	transform.setRotation(quat);
			// 	vive_t265.setRotation(vquat);
			// 	// vive_t265.inverse();

			// 	// static_transform.transform.rotation.x = quat.x();
			// 	// static_transform.transform.rotation.y = quat.y();
			// 	// static_transform.transform.rotation.z = quat.z();
			// 	// static_transform.transform.rotation.w = quat.w();

			// 	// transform.mult(transform.inverse(), vive_t265.inverse());
			// 	// transform.inverse();

			// 	static_transform.header.frame_id = "vive_pose";
			// 	static_transform.child_frame_id = "vive_odom";

			// 	static_transform.transform.translation.x = transform.getOrigin().getX();
			// 	static_transform.transform.translation.y = transform.getOrigin().getY();
			// 	static_transform.transform.translation.z = transform.getOrigin().getZ();

			// 	static_transform.transform.rotation.x = transform.getRotation().getX();
			// 	static_transform.transform.rotation.y = transform.getRotation().getY();
			// 	static_transform.transform.rotation.z = transform.getRotation().getZ();
			// 	static_transform.transform.rotation.w = transform.getRotation().getW();

			// 	static_broadcaster.sendTransform(static_transform);
			// }

			nav_msgs::Odometry gt;
			nav_msgs::Odometry b_gt;
			tf::Matrix3x3 rotate, rraw, identity;
			identity.setValue(1,0,0,
							  0,1,0,
							  0,0,1);
			rotate.setValue(tck.pose[0][0],tck.pose[0][1],tck.pose[0][2],
							tck.pose[1][0],tck.pose[1][1],tck.pose[1][2],
							tck.pose[2][0],tck.pose[2][1],tck.pose[2][2]);
			rraw.setValue(tck.poseRaw[0][0],tck.poseRaw[0][1],tck.poseRaw[0][2],
						tck.poseRaw[1][0],tck.poseRaw[1][1],tck.poseRaw[1][2],
						tck.poseRaw[2][0],tck.poseRaw[2][1],tck.poseRaw[2][2]);
			// printf("%f %f %f\n%f %f %f\n%f %f %f\n", rotate[0][0],rotate[0][1],rotate[0][2]
			// 									    , rotate[1][0],rotate[1][1],rotate[1][2]
			// 									    , rotate[2][0],rotate[2][1],rotate[2][2]);
			tf::Quaternion b_gt_quat, id_quat;
			geometry_msgs::Quaternion gt_qmsg, b_gt_qmsg, id_qmsg;
			double r,p,y,r1,p1,y1,r_gt,p_gt,y_gt;
			rotate.getRPY(r,p,y);
			rraw.getRPY(r1,p1,y1);
			rotate.getRotation(gt_quat);
			identity.getRotation(id_quat);
			// gt_quat.inverse();
			quaternionTFToMsg(gt_quat, gt_qmsg);
			quaternionTFToMsg(id_quat, id_qmsg);

			b_gt_quat.setRPY(0,0,cg.psi);
			quaternionTFToMsg(b_gt_quat, b_gt_qmsg);
			//for output
			tf::Matrix3x3(b_gt_quat).getRPY(r_gt,p_gt,y_gt);
			


			gt.header.stamp = current_time;
			gt.header.frame_id = "vive_odom";
			gt.child_frame_id = "vive_pose";

			//set the position
			gt.pose.pose.position.x = tck.pose[0][3];
			gt.pose.pose.position.y = tck.pose[1][3];
			gt.pose.pose.position.z = tck.pose[2][3];
			gt.pose.pose.orientation = gt_qmsg;
			gt.pose.covariance[0] = 0.1;
			gt.pose.covariance[7] = 0.1;
			gt.pose.covariance[14] = 0.1;
			gt.pose.covariance[21] = 0.001;
			gt.pose.covariance[28] = 0.001;
			gt.pose.covariance[35] = 0.001;

			//set the velocity
			gt.twist.twist.linear.x = tck.linVel[0];
			gt.twist.twist.linear.y = tck.linVel[1];
			gt.twist.twist.linear.z = tck.linVel[2];

			//set the velocity
			gt.twist.twist.angular.x = tck.angVel[0];
			gt.twist.twist.angular.y = tck.angVel[1];
			gt.twist.twist.angular.z = tck.angVel[2];
			gt.twist.covariance[0] = 0.1;
			gt.twist.covariance[7] = 0.1;
			gt.twist.covariance[14] = 0.1;
			gt.twist.covariance[21] = 0.001;
			gt.twist.covariance[28] = 0.001;
			gt.twist.covariance[35] = 0.001;

			gt_pub.publish(gt);

			b_gt.header.stamp = current_time;
			b_gt.header.frame_id = "map";
			b_gt.child_frame_id = "base_link";
			//set the position
			b_gt.pose.pose.position.x = cg.x;
			b_gt.pose.pose.position.y = cg.y;
			b_gt.pose.pose.position.z = 0;
			b_gt.pose.pose.orientation = b_gt_qmsg;
			b_gt.pose.covariance[0] = 0.1;
			b_gt.pose.covariance[7] = 0.1;
			b_gt.pose.covariance[14] = 0.1;
			b_gt.pose.covariance[21] = 0.001;
			b_gt.pose.covariance[28] = 0.001;
			b_gt.pose.covariance[35] = 0.001;

			//set the velocity
			b_gt.twist.twist.linear.x = cg.dx[0];
			b_gt.twist.twist.linear.y = cg.dy[0];
			b_gt.twist.twist.linear.z = 0;

			//set the velocity
			b_gt.twist.twist.angular.x = tck.angVel[0];
			b_gt.twist.twist.angular.y = tck.angVel[1];
			b_gt.twist.twist.angular.z = tck.angVel[2];
			b_gt.twist.covariance[0] = 0.1;
			b_gt.twist.covariance[7] = 0.1;
			b_gt.twist.covariance[14] = 0.1;
			b_gt.twist.covariance[21] = 0.001;
			b_gt.twist.covariance[28] = 0.001;
			b_gt.twist.covariance[35] = 0.001;

			b_gt_pub.publish(b_gt);

			if (cnt%100 == 0){
				printf("sec: %d ", (int)sec);
				printf("lsec: %d \n", (int)lsec);
				// showData(tck);
				// printf("\n");
				printf("pose:\n");
				printf("  raw: %f %f %f\n", tck.poseRaw[0][3],tck.poseRaw[1][3],tck.poseRaw[2][3]);
				printf("  trs: %f %f %f\n", tck.pose[0][3],tck.pose[1][3],tck.pose[2][3]);
				printf("   gt: %f %f %f\n", gt.pose.pose.position.x,gt.pose.pose.position.y,gt.pose.pose.position.z);
				//printf("  gt yaw: %f\n", tf::getYaw(gt.pose.pose.orientation));
				printf("  gt roll: %f, gt pitch: %f, gt yaw: %f\n", r,p,y);
				printf("  base_gt_odom: %f %f %f\n", b_gt.pose.pose.position.x,b_gt.pose.pose.position.y,b_gt.pose.pose.position.z);
				printf("  base_gt_odom roll: %f, pitch: %f, yaw: %f\n", r_gt,p_gt,y_gt);
				// printf(" odom: %f %f %f\n", odom_trans.getX(), odom_trans.getY(), odom_trans.getZ());
				// printf(" cali: %f %f %f\n", cali_trans.getX(), cali_trans.getY(), cali_trans.getZ());
				// printf("origin %f\n",sqrt((pow(odom_trans.getX(),2)+pow(odom_trans.getY(),2))/2));
				// printf("tru: %d\n", tru_vive);
				// printf("on: %d, tru: %d, cali: %d\n", vive_on, tru_vive, is_cali);
				// printf("   gt: %f %f %f\n", gt.pose.pose.position.x,gt.pose.pose.position.y,gt.pose.pose.position.z);
				// printf(" l_gt: %f %f %f\n", last_gt.pose.pose.position.x,last_gt.pose.pose.position.y,last_gt.pose.pose.position.z);
				// printf("ang:\n");
				// printf("  raw: %f %f %f\n", r1,p1,y1);
				// printf("  tan: %f %f %f\n", r,p,y);
				// printf("\n");
				// std::cout << "trs:" << r << " " << p << " " << y << std::endl;
				// printf("trs:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", tck.pose[0][0],tck.pose[0][1],tck.pose[0][2],tck.pose[0][3]
				// 								    				  , tck.pose[1][0],tck.pose[1][1],tck.pose[1][2],tck.pose[1][3]
				// 								    				  , tck.pose[2][0],tck.pose[2][1],tck.pose[2][2],tck.pose[2][3]);
				// std::cout << "raw:" << r1 << " " << p1 << " " << y1 << std::endl;
				// printf("raw:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", tck.poseRaw[0][0],tck.poseRaw[0][1],tck.poseRaw[0][2],tck.poseRaw[0][3]
				// 								    				  , tck.poseRaw[1][0],tck.poseRaw[1][1],tck.poseRaw[1][2],tck.poseRaw[1][3]
				// 								    				  , tck.poseRaw[2][0],tck.poseRaw[2][1],tck.poseRaw[2][2],tck.poseRaw[2][3]);
			// 	end = std::chrono::high_resolution_clock::now();
			// 	duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
			// 	printf(",  current routine used %lf ms.\n\n", duration.count()/1000000.0);
			}
			cnt ++;
			sec = cnt*TS;
		}
	private:
		ros::NodeHandle n;
		ros::Timer timer;
		ros::Time current_time;
		ros::Publisher gt_pub;
		ros::Publisher b_gt_pub;
		tf::TransformListener listener;
		tf::StampedTransform vive_t265;
		tf::StampedTransform transform;
		geometry_msgs::TransformStamped static_transform;
		tf2_ros::StaticTransformBroadcaster static_broadcaster;
		bool vive_on;
		bool tru_vive;
		bool is_cali;
		nav_msgs::Odometry last_gt;
		ros::Subscriber t265_odom_sub;
		tf::Quaternion cali_quat;
		tf::Matrix3x3 cali_rotate;
		tf::Vector3 cali_trans;
		tf::Quaternion odom_quat;
		tf::Matrix3x3 odom_rotate;
		tf::Vector3 odom_trans;
		tf::Quaternion gt_quat;
		tf::Matrix3x3 gt_rotate;
		tf::Vector3 gt_trans;
};


// void timerCallback(const ros::TimerEvent& e){
// 	// get tracker data from shm //
// 	P(semid);
// 	shm2tck(shmPtr, tckPtr);
// 	V(semid);
// 	if(!tck.valid){
// 		printf("[Lose connect with the tracker.]\n");
// 		lcnt++;
// 		lsec = lcnt*TS;
// 		// emergencyStop(&timer);
// 	}

// 	// tck2cg(tck, cgPtr);      // calculate data of cg
// 	// cg2tires(cg, tiresPtr);  // calculate data of tires

// 	if (cnt%100 == 0){
// 		printf("sec: %d ", (int)sec);
// 		printf("lsec: %d \n", (int)lsec);
// 		showData(tck);
				
// 		current_time = ros::Time::now();

// 		nav_msgs::Odometry gt;
// 		tf::Matrix3x3 rotate;
// 		rotate.setValue(tck.pose[0][0],tck.pose[0][1],tck.pose[0][2],
// 						tck.pose[1][0],tck.pose[1][1],tck.pose[1][2],
// 						tck.pose[2][0],tck.pose[2][1],tck.pose[2][2]);
// 		// printf("%f %f %f\n%f %f %f\n%f %f %f\n", rotate[0][0],rotate[0][1],rotate[0][2]
// 		// 										, rotate[1][0],rotate[1][1],rotate[1][2]
// 		// 										, rotate[2][0],rotate[2][1],rotate[2][2]);
// 		tf::Quaternion gt_quat;
// 		geometry_msgs::Quaternion gt_qmsg;
// 		rotate.getRotation(gt_quat);
// 		quaternionTFToMsg(gt_quat, gt_qmsg);

// 		gt.header.stamp = current_time;
// 		gt.header.frame_id = "ground_truth";
// 		//set the position
// 		gt.pose.pose.position.x = tck.pose[0][3];
// 		gt.pose.pose.position.y = tck.pose[1][3];
// 		gt.pose.pose.position.z = tck.pose[2][3];
// 		gt.pose.pose.orientation = gt_qmsg;

// 		//set the velocity
// 		gt.twist.twist.linear.x = tck.linVel[0];
// 		gt.twist.twist.linear.y = tck.linVel[1];
// 		gt.twist.twist.linear.z = tck.linVel[2];

// 		//set the velocity
// 		gt.twist.twist.angular.x = tck.angVel[0];
// 		gt.twist.twist.angular.y = tck.angVel[1];
// 		gt.twist.twist.angular.z = tck.angVel[2];

// 		gt_pub.publish(gt);

// 	// 	end = std::chrono::high_resolution_clock::now();
// 	// 	duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
// 	// 	printf(",  current routine used %lf ms.\n\n", duration.count()/1000000.0);
// 	}
// 	cnt ++;
// 	sec = cnt*TS;
// }

int main(int argc, char **argv){

	ros::init(argc, argv, "tracker");
	Tracker tracker;

	char   *T0invStr = (char*)malloc(sizeof(char)*256);
	ROS_INFO("Start");

	tckPtr = &tck;
	cgPtr = &cg;

	// ================== read inverse origin ===================
	memset(T0invStr, 0, sizeof(char)*256);
	// if (readT0inv("/home/lab816/agv_ws/src/sdc/scripts/T0inv.txt", T0invStr) == 0){
	if (readT0inv("/home/lab816/high_precision_amr/controller/my_amm_demo/py/T0inv.txt", T0invStr) == 0){
		printf("cannot read scripts/T0inv.txt\n");
		return 0;
	}
	//printf("%s",T0invStr);
	if (str2originInv(T0invStr, tck.originInv) == 0){
		printf("error: str2originInv()\n");
		return 0;
	}
	free(T0invStr);
	printf("read inverse origin successfully:\n");
	for(int m=0; m<3; m++){
		printf("\t");
		for(int n=0; n<4; n++){
			printf("%lf\t",tck.originInv[m][n]);
		}
		printf("\n");
	}

	// ======================== shm init ========================
	if((shmid = shmget(shmKey, page_size, IPC_CREAT | 0666)) == -1){
		printf("shmget error\n");
		exit(1);
	}
	if((shmPtr = shmat(shmid, NULL, 0)) == (void*)-1){
		printf("shmat error\n");
		exit(1);
	}
	
	// ======================== sem init ========================
	if((semid = semget(semKey, 1, IPC_CREAT | 0666)) == -1){
		printf("semget error\n");
		exit(1);
	}

	// ======================= timer init =======================
	// memset(&sa, 0, sizeof(sa));
	// sa.sa_handler = &timerRoutine;
	// sigaction(SIGALRM, &sa, NULL);
	// timer.it_value.tv_sec = 0;
	// timer.it_value.tv_usec = TS*1000000;
	// timer.it_interval.tv_sec = 0;
	// timer.it_interval.tv_usec = TS*1000000;

	// ======================= start timer ======================
	// setitimer(ITIMER_REAL, &timer, NULL);
	// timer = n.createTimer(ros::Duration(0.01), timerCallback);

	ros::spin();
	printf("Start Vive\n");

	return 1;
}

//========================================================================
//                          semaphore operations 
//========================================================================
// aquire (try to decrease)
int P(int s){
	sop.sem_num = 0;
	sop.sem_op = -1;
	sop.sem_flg = 0;
	
	if(semop(s, &sop, 1) < 0){
		printf("semop error: (P(sem))\n");
		return -1;
	}
	return 0;
}

// release (increase)
int V(int s){
	sop.sem_num = 0;
	sop.sem_op = 1;
	sop.sem_flg = 0;
	
	if(semop(s, &sop, 1) < 0){
		printf("semop error: (V(sem))\n");
		return -1;
	}
	return 0;
}