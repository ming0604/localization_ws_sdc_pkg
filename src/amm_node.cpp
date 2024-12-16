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
// #include <time.h>
// #include <signal.h>
// #include <sys/time.h>
// #include <chrono>

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
#include "upperLayer.h"
#include "kkt.h"
#include "lowerLayer.h"
#include "/opt/advantech/examples/C++_Console/inc/compatibility.h"
#include "/opt/advantech/inc/bdaqctrl.h"

#define  deviceDescription1884_0  L"PCIE-1884,BID#0"
#define  deviceDescription1884_1  L"PCIE-1884,BID#1"
#define  deviceDescription1730    L"PCI-1730,BID#0"
#define  DUTY_CYCLE               0.002 // 500 Hz PWM
#define  DEADZONE                 0.005 // 0.5% deadzone

// #define L	1.5
// #define Vd	1.2

// #define  L   7.0
#define L	14.0
#define Vd	1.4
#define R	6.0 
// #define End 1.0
// #define End 3.0
// #define End 3.5 // straight line
// #define End 4.0 // turn
// #define End 4.5
// #define End 5.0 // less error
// #define End 6.0 
// #define End 7.0
// #define End 7.5
// #define End 8.0
#define End 8.5
// #define End 9.0
// #define End 10.0
// #define End 10.5
// #define End 11.0
#define Sensor 0 //tck: 0, amcl: 1
#define Test 0 //straight: 0, Jturn: 1

#define  WRITE_FILE  true
#define  FILE_DIR    "/home/lab816/agv_ws/src/sdc/exp/1021/"

using namespace Automation::BDaq;

// ================= enum ======================
enum LOWER_STATE{OPENLOOP=0, EST_ON, CTRL_ON, CLOSELOOP};
#define Vth 0.4 // threshold of states switching

// ================ globals ====================
char   		  c;  // use to catch emergencyStop
pthread_t 	  id;
unsigned int  cnt=0;
void          *shmPtr = NULL;
int           shmid;
int           semid;
key_t         shmKey = (key_t)666;
key_t         semKey = (key_t)777;
struct sembuf sop; // for P() V() operation
long          page_size = sysconf(_SC_PAGESIZE);
char          *chPtr;
double        sec=0;
double        k_gain[1751][8];
double        dk_gain[1751][8];
double        k_gain_plus[1751][8];
double        dk_gain_plus[1751][8];
double    	  Pe[1751][6];
double 		  b[1751];
int           r_len = 1751;
double  	  M1[1851][9];
double		  cg1[1851][21];
double		  cg2[1851][21];
double		  ul1[1851][24];
double		  F1[1851][15];
double		  tck1[1851][7];
double		  amcl1[1851][14];
double		  odom1[1851][3];
int 		  time_len = 1851;

char fileName[80];
char subName[10];

// struct sigaction sa;
// struct itimerval timer;
ros::Timer timer;
int amcl_flag = 0;

FILE      **fPtrArr; // fPtrArr[0] => record tire 1~4 signals
FILE      *cgfPtr;
FILE		  *cgafPtr;
FILE      *ulfPtr;
FILE  	  *FfPtr;
FILE  		*MfPtr;
FILE 		  *M1fPtr;
FILE 		  *SLMP;
FILE 		  *KKfPtr;
FILE 		  *dKKfPtr;
FILE 		  *KKplusfPtr;
FILE      *dKKplusfPtr;
FILE		  *tckfPtr;
FILE		  *amclfPtr;
FILE		  *odomfPtr;

ErrorCode        ret = Success;
PwModulatorCtrl  *pwModulatorCtrl = PwModulatorCtrl::Create();
UdCounterCtrl    *udCounterCtrl = UdCounterCtrl::Create();
InstantDoCtrl    *instantDoCtrl = InstantDoCtrl::Create();
Array<PoChannel> *poChannel;
Array<UdChannel> *udChannel;

const bool forwordDirection[4] = {true,false,true,false};
PulseWidth pulseWidth[4];
uint8      direction[4] = {1,0,1,0};
double     voltage[4] = {0.0};

TRACKER          tck, *tckPtr;
AMCLPOSE		 		 amcl, *amclPtr;
ODOMPOSE		 		 odom, *odomPtr;
CG               cg, *cgPtr;
CG               cga, *cgaPtr;
TIRES            tires, *tiresPtr;
UPPERLAYER_DATA  upperLayerData, *upperLayerDataPtr;
KKT_DATA         kktData, *kktDataPtr;
ESTIMATOR_DATA   estimatorData[4];
CONTROLLER_DATA  controllerData[4];
LOWER_STATE      lowerState = OPENLOOP;


// std::chrono::high_resolution_clock::time_point start;
// std::chrono::high_resolution_clock::time_point end;
// std::chrono::nanoseconds duration;

/*temperary testing variables*/
/*
double slop, secLen, t1, t2, t3, spdMax;
double turnRate = 0.0;
double FadMax = 100.0;
double T[4] = {0.0};
double Fad[4] = {0.0};
double spd[4] = {0.0};
*/

inline void outPutTxt(void);
int idle(void);
void *emergencyCatch(void *arg);
int emergencyStop(void);
int daqInit(void); // initialization of PWM, QEP, DO
int voltage2pulseWidth(double voltage[4], PulseWidth *pulseWidth, uint8 direction[4]);
int outputPwmDir(PulseWidth pulseWidth[4], Array<PoChannel> *poChannel, uint8 direction[4], InstantDoCtrl *instantDoCtrl);
int getTiresOmega(TIRES *tPtr);
inline void T2voltage(CONTROLLER_DATA ctrl[4], TIRES *t, double voltage[4]);
int P(int s);
int V(int s);
int speedCtrl(double v[4], double spd[4], TIRES *t);

void lowerLayerStateMachine(void);
void assignForce(double Fad[4], double assignVal, double sec, double t1, double t2);

void read_K(double k_gain[][8],int r_len);
void read_dK(double dk_gain[][8],int r_len);
void read_Pe(double Pe[][6],int r_len);
void read_b(double b[],int r_len);

void write_KK(FILE *KKfPtr,double k_gain[][8],int r_len);
void write_M1(FILE *M1fPtr,double M1[][9],int time_len);
void write_cg(FILE *cgfPtr,double cg1[][21],int time_len);
void write_ul(FILE *ulfPtr,double ul1[][24],int time_len);
void write_F(FILE *FfPtr,double F1[][15],int time_len);
void write_dKKplus(FILE *dKKplusfPtr,double dk_gain_plus[][8],int r_len);
void write_KKplus(FILE *KKplusfPtr,double k_gain_plus[][8],int r_len);
void write_tck(FILE *tckfPtr,double tck1[][7],int time_len);
void write_amcl(FILE *amclfPtr,double amcl1[][14],int time_len);
void write_odom(FILE *odomfPtr,double odom1[][3],int time_len);

// ros::Subscriber amcl_sub;
// ros::Subscriber odom_sub, vive_sub;
// void amclReceived(const nav_msgs::OdometryConstPtr& msg);
void odomReceived(const nav_msgs::OdometryConstPtr& msg);
// void viveReceived(const nav_msgs::OdometryConstPtr& msg);

class AMMNode{
  public:
	AMMNode(ros::NodeHandle* nodehandle_1, ros::NodeHandle* nodehandle_2);
	// AMMNode(ros::NodeHandle* nodehandle_1);
    ~AMMNode();
	void amclReceived(const nav_msgs::OdometryConstPtr& msg);
	void timerCallback(const ros::TimerEvent& e);
	void viveReceived(const nav_msgs::OdometryConstPtr& msg);
	// int emergencyStop();
  private:
	ros::NodeHandle nh_amcl;
	ros::NodeHandle nh_amm;
	ros::NodeHandle pn;
	ros::Subscriber amcl_sub;
	ros::Subscriber vive_sub;
	// ros::Timer timer, emergency_timer;

	double start, end, duration;
	double vive_pose[3];
};

// thread
void *emergencyCatch(void *arg){
	ROS_INFO("Catch thread");

	// id = pthread_self();
	// std::cout << "catch: " << id << std::endl;
	id = getpid();
	ROS_INFO("emergencyCatch ID: %lu", id);

	while(true){
		// std::cout << "catch emergencyStop" << std::endl;
		c = getchar();
		if ( c=='\n' || (c>'a' && c<'z') || (c>'A' && c<'Z')){
			emergencyStop();
#if WRITE_FILE
			ros::Duration(1.0).sleep();
			// sleep(1);
			for(int i=0;i<4;i++){fclose(fPtrArr[i]);}
			//fclose(cgfPtr);
			//fclose(ulfPtr);
			//fclose(FfPtr);
			//fclose(MfPtr);

			free(fPtrArr);

#endif //#if WRITE_FILE
			break;
		}
	}
	ROS_INFO("Exit thread");
	pthread_exit(NULL);
}

AMMNode::AMMNode(ros::NodeHandle* nodehandle_1, ros::NodeHandle* nodehandle_2):nh_amm(*nodehandle_1),nh_amcl(*nodehandle_2){
// AMMNode::AMMNode(ros::NodeHandle* nodehandle_1):pn(*nodehandle_1){
	ROS_INFO("AMMNode");

	// vive_sub = pn.subscribe("gt", 2, &AMMNode::viveReceived, this);
	// amcl_sub = pn.subscribe("amcl_ekf", 2, &AMMNode::amclReceived, this);
	// timer = pn.createTimer(ros::Duration(0.01), &AMMNode::timerCallback, this);

	amcl_sub = nh_amcl.subscribe("amcl_ekf", 500, &AMMNode::amclReceived, this);
	timer = nh_amm.createTimer(ros::Duration(0.01), &AMMNode::timerCallback, this);
		
	// ros::WallDuration(1.0).sleep();

	// ros::spin();
}

AMMNode::~AMMNode(){

}

void AMMNode::amclReceived(const nav_msgs::OdometryConstPtr& msg){
// void amclReceived(const nav_msgs::OdometryConstPtr& msg){
	ROS_INFO("amclReceived");
	ROS_INFO("amcl_t: %ld", msg->header.stamp.toNSec());
	// amclPtr->time = ros::Time::now().toNSec();
	// printf("amcl_t: %lf\n", (amcl.time - 1.6769641e+18));
	amclPtr->pose[0] = msg->pose.pose.position.x;
	amclPtr->pose[1] = msg->pose.pose.position.y;
	amclPtr->pose[2] = tf2::getYaw(msg->pose.pose.orientation);

	amclPtr->ekf_v[0] = msg->twist.twist.linear.x;
	amclPtr->ekf_v[1] = msg->twist.twist.linear.y;
	amclPtr->ekf_v[2] = msg->twist.twist.angular.z;

	amcl_flag = 1;
}

void odomReceived(const nav_msgs::OdometryConstPtr& msg){
	ROS_INFO("odomReceived");
	odomPtr->pose[0] = msg->pose.pose.position.x;
	odomPtr->pose[1] = msg->pose.pose.position.y;
	odomPtr->pose[2] = tf2::getYaw(msg->pose.pose.orientation);

	odomPtr->linVel[0] = msg->twist.twist.linear.x;
	odomPtr->linVel[1] = msg->twist.twist.linear.y;
	odomPtr->angVel[2] = msg->twist.twist.angular.z;
}

void AMMNode::viveReceived(const nav_msgs::OdometryConstPtr& msg){
// void viveReceived(const nav_msgs::OdometryConstPtr& msg){
	ROS_INFO("viveReceived");
	tckPtr->pose[0][3] = msg->pose.pose.position.x;
	tckPtr->pose[1][3] = msg->pose.pose.position.y;
	// tckPtr->pose[2][3] = tf2::getYaw(msg->pose.pose.orientation);

	tckPtr->linVel[0] = msg->twist.twist.linear.x;
	tckPtr->linVel[1] = msg->twist.twist.linear.y;
	tckPtr->angVel[2] = msg->twist.twist.angular.z;
}

void AMMNode::timerCallback(const ros::TimerEvent& e){
	// ROS_INFO("%d", amcl_flag);

	if (amcl_flag == 0){
		ROS_INFO("WAIT");
		return ;
	}else {
		ROS_INFO("timerCallback");
		ROS_INFO("cur exp.: %ld, cur real: %ld", e.current_expected.toNSec(), e.current_real.toNSec());

		start = ros::Time::now().toNSec();
		// std::cout << e.last_real << std::endl;

		voltage2pulseWidth(voltage, pulseWidth, direction);
		outputPwmDir(pulseWidth, poChannel, direction, instantDoCtrl);

		getTiresOmega(tiresPtr); // get tires' angular velocity

		// Vivecode
		//-- get tracker data from shm --//
		P(semid);
		shm2tck(shmPtr, tckPtr);
		V(semid);

		if(!tck.valid){
			printf("[Lose connect with the tracker.]\n");
			emergencyStop();

			ROS_WARN("Tracker losing");
			exit(1);
		}

		if(Sensor){
			signal2cg(tck, amclPtr, cgPtr, cgaPtr, sec);
			// amcl2cg(amclPtr, cgaPtr);
			cg2tires(cga, tiresPtr);  // calculate data of tires
		}else{
			tck2cg(tck, cgPtr);      // calculate data of cg
			amcl2cg(amclPtr, cgaPtr);
			cg2tires(cg, tiresPtr);  // calculate data of tires
		}
		printf("%f, %f, %f\n", cg.dx[0], cga.dx[0], amcl.ekf_vxl[0]);

		// tck2cg(tck, cgPtr);      // calculate data of cg
		// amcl2cg(amcl, cgPtr);
		
		// if(!tck.valid){
		// 	amcl2cg(amcl, cgPtr);
		// }else{
		// 	tck2cg(tck, cgPtr);
		// }

		// if(sec < 2.5)
		// 	tck2cg(tck, cgPtr);
		// else
		// 	amcl2cg(amcl, cgPtr);

		// printf("ammt: %f\n", amcl.time);
		printf("  cg: %f, %f\n", cg.x, cg.y);
		printf(" tck: %f, %f\n", tck.pose[0][3], tck.pose[1][3]);
		printf("amcl: %f, %f\n", amcl.pose[0], amcl.pose[1]);
		printf("odom: %f, %f\n\n", odom.pose[0], odom.pose[1]);

		if(Test){
			genDesiredTrajectory8(sec, upperLayerDataPtr, L, Vd, R);
		}else{
			genDesiredTrajectory7(sec, upperLayerDataPtr, L, Vd);//genDesiredTrajectory6(sec, upperLayerDataPtr, L, v)
		}
	
		if(Sensor){
			if(Test){
				calculate_road_error_genDesiredTrajectory8(cga,upperLayerDataPtr,sec,L,Vd,R);//calculate_road_error_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double sec,double L, double v,double R);
				upperLayerControl_genDesiredTrajectory8(cgaPtr, upperLayerDataPtr,k_gain,dk_gain,k_gain_plus,dk_gain_plus,Pe,b,sec,L,Vd,R); //int upperLayerControl_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double k_gain[][8],double dk_gain[][8],double sec,double L, double v,double R);
			}else{
				calculate_road_error_genDesiredTrajectory7(cga,upperLayerDataPtr,sec,L,Vd);
				upperLayerControl(cgaPtr, upperLayerDataPtr,k_gain,dk_gain,Pe,b,sec,L,Vd);
			}
		}else{
			if(Test){
				calculate_road_error_genDesiredTrajectory8(cg,upperLayerDataPtr,sec,L,Vd,R);//calculate_road_error_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double sec,double L, double v,double R);
				upperLayerControl_genDesiredTrajectory8(cgPtr, upperLayerDataPtr,k_gain,dk_gain,k_gain_plus,dk_gain_plus,Pe,b,sec,L,Vd,R); //int upperLayerControl_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double k_gain[][8],double dk_gain[][8],double sec,double L, double v,double R);
			}else{
				calculate_road_error_genDesiredTrajectory7(cg,upperLayerDataPtr,sec,L,Vd);
				upperLayerControl(cgPtr, upperLayerDataPtr,k_gain,dk_gain,Pe,b,sec,L,Vd);
			}
		}

		// cg2tires(cg, tiresPtr);  // calculate data of tires

		// genDesiredTrajectory7(sec, upperLayerDataPtr, 5, 1.8);//genDesiredTrajectory6(sec, upperLayerDataPtr, L, v)
		// calculate_road_error_genDesiredTrajectory7(cg,upperLayerDataPtr,sec,5,1.8);
		// upperLayerControl(cgPtr, upperLayerDataPtr,k_gain,dk_gain,Pe,b,sec,3,1.2);

		// genDesiredTrajectory8(sec, upperLayerDataPtr, L, Vd, R);
		
		// calculate_road_error_genDesiredTrajectory8(cg,upperLayerDataPtr,sec,L,Vd,R);//calculate_road_error_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double sec,double L, double v,double R);

		// upperLayerControl_genDesiredTrajectory8(cgPtr, upperLayerDataPtr,k_gain,dk_gain,k_gain_plus,dk_gain_plus,Pe,b,sec,L,Vd,R); //int upperLayerControl_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double k_gain[][8],double dk_gain[][8],double sec,double L, double v,double R);

		if(kktOptimization(upperLayerData.FX,upperLayerData.M, kktDataPtr) == 0){
			emergencyStop();
			for(int i=0;i<4;i++){fclose(fPtrArr[i]);}
			//close(cgfPtr);
			printf("FX:%f, M:%f\n",upperLayerData.FX, upperLayerData.M);
			
			ROS_WARN("KKT constraint");
			ros::shutdown();
			exit(1);
		}
		if((voltage[0] == 24) || (voltage[1] == 24) || (voltage[2] == 24) || (voltage[3] == 24)){
			emergencyStop();
			printf("[Voltage 24]\n");
			for(int index=0; index<4; index++){
				voltage[index] = 0.0;
			}
			ROS_WARN("Voltage constraint");
			ros::shutdown();
			exit(1);
		}

		lowerLayerStateMachine();

		T2voltage(controllerData, tiresPtr, voltage);

		if(!(sec<End)){
			ROS_WARN("Over times");
			emergencyStop();
			for(int index=0; index<4; index++){
				voltage[index] = 0.0;
			}
		}
		/////////////////////////////////////////////save data in M1
		M1[cnt][0]=sec;
		M1[cnt][1]=upperLayerData.M;
		M1[cnt][2]=upperLayerData.M_1;
		M1[cnt][3]=upperLayerData.M_2;
		M1[cnt][4]=upperLayerData.M_3;
		M1[cnt][5]=upperLayerData.M_D2;
		M1[cnt][6]=upperLayerData.M_ks2sat;
		M1[cnt][7]=upperLayerData.M_lemda2SV2;
		M1[cnt][8]=upperLayerData.r_r[0];
		// /////////////////////////////////////////////save data in cg
		cg1[cnt][0]=sec;
		cg1[cnt][1]=cg.x;
		cg1[cnt][2]=cg.y;
		cg1[cnt][3]=cg.psi;
		cg1[cnt][4]=cg.dx_r_LPF[0];
		cg1[cnt][5]=cg.VXLPF[0];
		cg1[cnt][6]=cg.VYLPF[0];
		cg1[cnt][7]=cg.rLPF[0];
		cg1[cnt][8]=cg.dr[0];
		cg1[cnt][9]=cg.AX;
		cg1[cnt][10]=lowerState;
		cg1[cnt][11]=voltage[0];
		cg1[cnt][12]=voltage[1];
		cg1[cnt][13]=voltage[2];
		cg1[cnt][14]=voltage[3];
		cg1[cnt][15]=cg.x_r[0];
		cg1[cnt][16]=cg.X;
		cg1[cnt][17]=cg.Y;
		cg1[cnt][18]=cg.dx[0];
		cg1[cnt][19]=cg.dy[0];
		cg1[cnt][20]=cg.ddx[0];

		cg2[cnt][0]=sec;
		cg2[cnt][1]=cga.x;
		cg2[cnt][2]=cga.y;
		cg2[cnt][3]=cga.psi;
		cg2[cnt][4]=cga.dx_r_LPF[0];
		cg2[cnt][5]=cga.VXLPF[0];
		cg2[cnt][6]=cga.VYLPF[0];
		cg2[cnt][7]=cga.rLPF[0];
		cg2[cnt][8]=cga.dr[0];
		cg2[cnt][9]=cga.AX;
		cg2[cnt][10]=lowerState;
		cg2[cnt][11]=voltage[0];
		cg2[cnt][12]=voltage[1];
		cg2[cnt][13]=voltage[2];
		cg2[cnt][14]=voltage[3];
		cg2[cnt][15]=cga.x_r[0];
		cg2[cnt][16]=cga.X;
		cg2[cnt][17]=cga.Y;
		cg2[cnt][18]=cga.dx[0];
		cg2[cnt][19]=cga.dy[0];
		cg2[cnt][20]=cga.ddx[0];
		// /////////////////////////////////////////////save data in ul
		ul1[cnt][0]=sec;
		ul1[cnt][1]=upperLayerData.SV1;
		ul1[cnt][2]=upperLayerData.SV2;
		ul1[cnt][3]=upperLayerData.FX;
		ul1[cnt][4]=upperLayerData.M;
		ul1[cnt][5]=upperLayerData.ccount;
		ul1[cnt][6]=upperLayerData.y_r_LPF[0];
		ul1[cnt][7]=upperLayerData.dy_r_LPF[0];
		ul1[cnt][8]=upperLayerData.psi_r_LPF[0];
		ul1[cnt][9]=upperLayerData.r_r[0];
		ul1[cnt][10]=upperLayerData.desiredTrajectory.xd;
		ul1[cnt][11]=upperLayerData.desiredTrajectory.yd;
		ul1[cnt][12]=upperLayerData.dxd_r[0];
		ul1[cnt][13]=upperLayerData.desiredTrajectory.psid;
		ul1[cnt][14]=upperLayerData.desiredTrajectory.rd;
		ul1[cnt][15]=upperLayerData.c_x;
		ul1[cnt][16]=upperLayerData.c_y;
		ul1[cnt][17]=upperLayerData.ddy_r_LPF[0];
		ul1[cnt][18]=upperLayerData.xd_r[0];
		ul1[cnt][19]=upperLayerData.dxd_r[0];
		ul1[cnt][20]=upperLayerData.law[0];
		ul1[cnt][21]=upperLayerData.law_dot[0];
		ul1[cnt][22]=upperLayerData.eta_last_matric;
		ul1[cnt][23]=upperLayerData.lemda1;
		// /////////////////////////////////////////////save data in F
		F1[cnt][0]=sec;
		F1[cnt][1]=upperLayerData.FX;
		F1[cnt][2]=-upperLayerData.ddxd_r;
		F1[cnt][3]=upperLayerData.F_D1;
		F1[cnt][4]=upperLayerData.F_ks1sat;
		F1[cnt][5]=upperLayerData.F_lemda1SV1;
		F1[cnt][6]=upperLayerData.F_dy_r_dx_r_curvature;
		F1[cnt][7]=estimatorData[0].mu_hat*estimatorData[0].Fz0*TIRE_RADIUS;
		F1[cnt][8]=estimatorData[1].mu_hat*estimatorData[1].Fz0*TIRE_RADIUS;
		F1[cnt][9]=estimatorData[2].mu_hat*estimatorData[2].Fz0*TIRE_RADIUS;
		F1[cnt][10]=estimatorData[3].mu_hat*estimatorData[3].Fz0*TIRE_RADIUS;
		F1[cnt][11]=kktData.Fad[0]*TIRE_RADIUS;
		F1[cnt][12]=kktData.Fad[1]*TIRE_RADIUS;
		F1[cnt][13]=kktData.Fad[2]*TIRE_RADIUS;
		F1[cnt][14]=kktData.Fad[3]*TIRE_RADIUS;
		// /////////////////////////////////////////////save data in tck, amcl
		tck1[cnt][0]=sec;
		tck1[cnt][1]=tck.pose[0][3];
		tck1[cnt][2]=tck.pose[1][3];
		tck1[cnt][3]=atan2(tck.pose[1][0], tck.pose[0][0]);
		tck1[cnt][4]=tck.linVel[0];
		tck1[cnt][5]=tck.linVel[1];
		tck1[cnt][6]=tck.angVel[2];

		amcl1[cnt][0]=sec;
		amcl1[cnt][1]=amcl.pose[0];
		amcl1[cnt][2]=amcl.pose[1];
		amcl1[cnt][3]=amcl.yaw[0];
		amcl1[cnt][4]=amcl.linVel_x[0];
		amcl1[cnt][5]=amcl.linVel_y[0];
		amcl1[cnt][6]=amcl.angVel_yaw[0];
		amcl1[cnt][7]=amcl.VXLPF[0];
		amcl1[cnt][8]=amcl.VYLPF[0];
		amcl1[cnt][9]=amcl.rLPF[0];
		amcl1[cnt][10]=amcl.ekf_vxl[0];
		amcl1[cnt][11]=amcl.ekf_vyl[0];
		amcl1[cnt][12]=amcl.ekf_v[2];
		amcl1[cnt][13]=amcl.imu;
		odom1[cnt][0]=sec;
		odom1[cnt][1]=odom.pose[0];
		odom1[cnt][2]=odom.pose[1];

		// end = ros::Time::now().toNSec();
		// duration = (end - start);
		// printf(",  current routine used %lf ms.\n", duration/1e+6);

		if (cnt%100==0){
			printf("sec: %d ", (int)sec);
			end = ros::Time::now().toNSec();
			duration = (end - start);
			printf(",  current routine used %lf ms.\n\n", duration/1e+6);
		}
		cnt ++;
		sec = cnt*TS;
		// amcl_flag = 0;
	}
// #if WRITE_FILE
	// outPutTxt();
// #endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AMMNode");
	
	ros::NodeHandle n;

	ros::Time::init();

	// char   c;  // use to catch emergencyStop
    // char   *T0invStr = (char*)malloc(sizeof(char)*256);

	// char fileName[80];
	// //const char dirName[] = FILE_DIR;
	// char subName[10];

#if WRITE_FILE
	if (argc != 2){
		printf("change args_1 in launch file\n");
		return 0;
	}

	fPtrArr = (FILE**)malloc(sizeof(FILE*)*4);

	for(int i=0;i<4;i++){
		memset(fileName,0,80);
		memset(subName,0,10);
		strcpy(fileName, FILE_DIR);
		strcat(fileName, argv[1]);
		sprintf(subName, "_%d.txt", i);
		strcat(fileName, subName);
		fPtrArr[i] = fopen(fileName, "w");
		if ( fPtrArr[i] != NULL) { printf("file opened: \"%s\"\n", fileName);}
		else {printf("open file \"%s\" failed.\n", fileName);return 0;}
	}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_cg.txt");
	cgfPtr = fopen(fileName, "w");
	if ( cgfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_cga.txt");
	cgfPtr = fopen(fileName, "w");
	if ( cgfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_ul.txt");
	ulfPtr = fopen(fileName, "w");
	if ( ulfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_F.txt");
	FfPtr = fopen(fileName, "w");
	if ( FfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_tck.txt");
	tckfPtr = fopen(fileName, "w");
	if ( tckfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_amcl.txt");
	amclfPtr = fopen(fileName, "w");
	if ( amclfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_odom.txt");
	odomfPtr = fopen(fileName, "w");
	if ( odomfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	// memset(fileName,0,80);
	// strcpy(fileName, FILE_DIR);
	// strcat(fileName, argv[1]);
	// strcat(fileName, "_M.txt");
	// MfPtr = fopen(fileName, "w");
	// if ( MfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	// else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_M.txt");
	M1fPtr = fopen(fileName, "w");
	if ( M1fPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_SLMP.txt");
	SLMP = fopen(fileName, "w");
	if ( SLMP != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_KK.txt");
	KKfPtr = fopen(fileName, "w");
	if ( KKfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_dKK.txt");
	dKKfPtr = fopen(fileName, "w");

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_KK_plus.txt");
	KKplusfPtr = fopen(fileName, "w");

	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName, argv[1]);
	strcat(fileName, "_dKK_plus.txt");
	dKKplusfPtr = fopen(fileName, "w");		
	if ( dKKfPtr != NULL) { printf("file opened: \"%s\"\n", fileName);}
	else {printf("open file \"%s\" failed.\n", fileName);return 0;}
#endif //#if WRITE_FILE

	/*temperary testing variables*/
	/*
	spdMax = 1.0;
	spdMax = spdMax/TIRE_RADIUS; // angular speed (rad/s)
	t1 = 1.0;
	t2 = 2.0;
	t3 = 3.0;
	*/

	tckPtr = &tck;
	amclPtr = &amcl;
	odomPtr = &odom;
	cgPtr = &cg;
	cgaPtr = &cga;
	tiresPtr = &tires;
	upperLayerDataPtr = &upperLayerData;
	kktDataPtr = &kktData;

	// ================== read k gain ===================
	// std::cout << "Read" << std::endl;
	ROS_INFO("Read K, dK, Pe, b");
  read_K(k_gain,r_len);
  read_dK(dk_gain,r_len);
  read_Pe(Pe,r_len);
  read_b(b,r_len);

	// std::cout << "Done" << std::endl ;

	// Vivecode, At tracker_n.cpp
	char   *T0invStr = (char*)malloc(sizeof(char)*256);
	// ================== read inverse origin ===================
	ROS_INFO("Vive origin, Shm, Sem");
	memset(T0invStr, 0, sizeof(char)*256);
	if (readT0inv("/home/lab816/agv_ws/src/sdc/scripts/T0inv.txt", T0invStr) == 0){
		printf("cannot read scripts/T0inv.txt\n");
		return 0;
	}
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

	
	ROS_INFO("DAQ, upperLayer, KKT, estimator initial");
	// ======================== DAQ init ========================
    if(daqInit() == 0){
        printf("error: daqInit()\n");
        return 0;
    }
	
	// ==================== upperLayer init =====================
	// std::cout << "upperlayer" << std::endl;
	memset(upperLayerDataPtr, 0, sizeof(UPPERLAYER_DATA));
	//upperLayerData.previewTrajectory.P = -0.1;

	// ======================== KKT init ========================
	kktData.u = 2.0;
	if(kktInit(kktDataPtr) == 0){
		return 0;
	}
	printf("kkt: |uFz0| = %f\n", sqrt(kktData.uFz0Square[0]));

	// ==================== estimator init ======================
	// std::cout << "estimatorInit" << std::endl;
	for(int index=0; index<4; index++){
		estimatorInit(estimatorData+index, index);
		//printf("index%d: e_factor %f\n",index,estimatorData[index].e_factor);
	}

    // ======================= timer init =======================
	// memset(&sa, 0, sizeof(sa));
	// sa.sa_handler = &timerRoutine;
	// sigaction(SIGALRM, &sa, NULL);
	// timer.it_value.tv_sec = 0;
	// timer.it_value.tv_usec = TS*1000000;
	// timer.it_interval.tv_sec = 0;
	// timer.it_interval.tv_usec = TS*1000000;

	// ========================== write SLMP KK dKK ==========================
	fprintf(SLMP,"%f %f %f %f %f %f %f %f %f %f\n",upperLayerData.ks1,ks2,C1,C2,upperLayerData.lemda1,upperLayerData.lemda2,epsilonSV1_U,epsilonSV1_L,epsilonSV2_U,epsilonSV2_L);

	fclose(SLMP);
	// ========================== idle ==========================
	ros::NodeHandle n_1, n_2;
	ros::CallbackQueue sub_queue, amm_queue, amcl_queue;

	n.setCallbackQueue(&sub_queue);
	n_1.setCallbackQueue(&amm_queue);
	n_2.setCallbackQueue(&amcl_queue);

	// amcl_sub = n.subscribe("/amcl_ekf", 500, &amclReceived);
	// odom_sub = n.subscribe("/t265/odom/sample", 2, &odomReceived);
	// vive_sub = n.subscribe("/gt", 2, &viveReceived);

	// AMMNode AMMNode(&n_1);
	// AMMNode AMMNode(&n_1, &n_2);

	ros::AsyncSpinner spinner(1, &sub_queue);
	spinner.start();
	
	// ros::AsyncSpinner s_2(1, &amcl_queue);
	// s_2.start();

	// ros::AsyncSpinner s_1(1, &amm_queue);

	// while(1){
	// 	ROS_INFO("%d", amcl_flag);
	// 	if(amcl_flag == 1){
	// 		ROS_INFO_STREAM("\033[1;32m---> Ready \033[0m");
	// 		break;
	// 	}
	// }

	idle();

	pthread_t catch_keyboard;
	int count = 1;
	if(pthread_create(&catch_keyboard, NULL, emergencyCatch, &count) != 0){
		printf("thread create error\n");
		exit(1);
	}
	// pthread_t mid = pthread_self();
	// std::cout << "main: " << mid << std::endl;
	// mid = getpid();
	// ROS_INFO("main ID: %lu", mid);

	// ======================= start timer ======================
	// setitimer(ITIMER_REAL, &timer, NULL);

	ros::AsyncSpinner s_2(1, &amcl_queue);
	s_2.start();

	// AMMNode AMMNode(&n_1);
	AMMNode AMMNode(&n_1, &n_2);

	// ros::Rate r(100);
	ros::AsyncSpinner s_1(2, &amm_queue);
	s_1.start();

	ros::waitForShutdown();

    // ================== catch emergencyStop ===================
// 	while(true){
// 		std::cout << "catch emergencyStop" << std::endl;
// 		c = getchar();
// 		if ( c=='\n' || (c>'a' && c<'z') || (c>'A' && c<'Z')){
// 			emergencyStop();
// #if WRITE_FILE
// 			sleep(1);
// 			for(int i=0;i<4;i++){fclose(fPtrArr[i]);}
// 			//fclose(cgfPtr);
// 			//fclose(ulfPtr);
// 			//fclose(FfPtr);
// 			//fclose(MfPtr);


// 			free(fPtrArr);

// #endif //#if WRITE_FILE
// 		}
// 	}

    return 1;
} // main()

// inline void outPutTxt(void){  // called by timerRoutine
// 	static int i;
// 	for(i=0;i<4;i++){
// 		fprintf(fPtrArr[i],"%f", tires.theTire[i].lambda);        //1
// 		fprintf(fPtrArr[i]," %f", tires.theTire[i].omegaLPF[0]);  //2
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].eI);           //3
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].ef);           //4
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].Emu);          //5
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].mu_hat);       //6
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].phi_tilde);    //7
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].omega_hat);    //8
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].omega_f_hat);  //9
// 		fprintf(fPtrArr[i]," %f", (controllerData+i)->T);         //10
// 		fprintf(fPtrArr[i]," %f", estimatorData[i].mu_hat*estimatorData[i].Fz0*TIRE_RADIUS); // 11 Fa_hat*R
// 		fprintf(fPtrArr[i]," %f", kktData.Fad[i]*TIRE_RADIUS);    //12
// 		fprintf(fPtrArr[i],"\n");
// 	}
// 	//fprintf(cgfPtr,"%f %f %f %f %f %f %f %f %f %f %d\n", sec, cg.x, cg.y, cg.psi, cg.dx_r, cg.VXLPF[0], cg.VYLPF[0], cg.rLPF[0], cg.dr[0], cg.AX, lowerState);
// 	//fprintf(ulfPtr,"%f %f %f %f %f %d %f %f %f %f %f %f %f %f %f %f %f\n",sec, upperLayerData.SV1, upperLayerData.SV2, upperLayerData.FX, upperLayerData.M,upperLayerData.ccount,upperLayerData.y_r[0],upperLayerData.dy_r_LPF[0],upperLayerData.psi_r[0],upperLayerData.r_r_LPF[0],upperLayerData.desiredTrajectory.xd,upperLayerData.desiredTrajectory.yd,upperLayerData.dxd_r,upperLayerData.desiredTrajectory.psid,upperLayerData.desiredTrajectory.rd,upperLayerData.c_x,upperLayerData.c_y);
// 	//fprintf(FfPtr,"%f %f %f %f %f %f %f\n",sec,upperLayerData.FX,-upperLayerData.ddxd_r,upperLayerData.F_D1,upperLayerData.F_ks1sat,upperLayerData.F_lemda1SV1,upperLayerData.F_dy_r_dx_r_curvature);
// 	//fprintf(MfPtr,"%f %f %f %f %f %f %f %f %f\n",sec,upperLayerData.M,upperLayerData.M_1, upperLayerData.M_2, upperLayerData.M_3, upperLayerData.M_D2, upperLayerData.M_ks2sat, upperLayerData.M_lemda2SV2,upperLayerData.r_r_LPF[0]);
// 	//fprintf(SLMP,"%f %f %f %f %f %f %f %f %f %f\n",ks1,ks2,C1,C2,lemda1,lemda2,epsilonSV1_U,epsilonSV1_L,epsilonSV2_U,epsilonSV2_L);
// }

int idle(void){
	char c;
	printf("idle...\n(Enter to start timer.)");
	while(true){
		c = getchar();
		if (c == '\n') break;
		scanf("%*[^\n]");
		getchar();
	}
	ROS_INFO_STREAM("\033[1;32m---> Running \033[0m");
	for(int i=0; i<4; i++){
		instantDoCtrl->WriteBit(2, i, 0); //enable motor
	}

	return 1;
}

int emergencyStop(void){
	ROS_WARN("EmergencyStop");
    for(int i=0;i<4;i++){
        pulseWidth[i].LoPeriod = 0.0;
        pulseWidth[i].HiPeriod = DUTY_CYCLE;
    }
	
	// stop timer
	// timerPtr->it_value.tv_sec = 0;
	// timerPtr->it_value.tv_usec = 0;
	// timerPtr->it_interval.tv_sec = 0;
	// timerPtr->it_interval.tv_usec = 0;
	// setitimer(ITIMER_REAL, timerPtr, NULL);
	timer.stop();

	// stop motor
	for(int i=0; i<4; i++){
		instantDoCtrl->WriteBit(2, i, 1); //disable motor
		poChannel->getItem(i).setPulseWidth(pulseWidth[i]);
	}
	write_M1(M1fPtr,M1,time_len);
	write_cg(cgfPtr,cg1,time_len);
	write_ul(ulfPtr,ul1,time_len);	
	write_F(FfPtr,F1,time_len);	
	write_tck(tckfPtr,tck1,time_len);
	write_amcl(amclfPtr,amcl1,time_len);
	write_odom(odomfPtr,odom1,time_len);
	write_KK(KKfPtr,k_gain,r_len);
	write_KK(dKKfPtr,dk_gain,r_len);

	write_KK(KKplusfPtr,k_gain_plus,r_len);
	write_KK(dKKplusfPtr,dk_gain_plus,r_len);

	fclose(KKfPtr);
	fclose(dKKfPtr);	
	fclose(M1fPtr);
	fclose(cgfPtr);
	fclose(ulfPtr);
	fclose(FfPtr);
	fclose(tckfPtr);
	fclose(amclfPtr);
	fclose(odomfPtr);
	fclose(dKKplusfPtr);
	fclose(KKplusfPtr);
	printf("Timer is stopped.\n");
  printf("current cnt: %d\n", cnt);
	printf("position:\n\tx: %.4f\n\ty: %.4f\n\tpsi: %.4f\n", cg.x, cg.y, cg.psi);
	ros::shutdown();

	return 1;
}

int daqInit(void){
	// std::cout << "DaqInit" << std::endl;
    for(int i=0;i<4;i++){
        pulseWidth[i].LoPeriod = 0.0;
        pulseWidth[i].HiPeriod = DUTY_CYCLE;
    }

    do{
		DeviceInformation devInfo1884_0(deviceDescription1884_0);
		DeviceInformation devInfo1884_1(deviceDescription1884_1);
		DeviceInformation devInfo1730(deviceDescription1730);
		ret = pwModulatorCtrl->setSelectedDevice(devInfo1884_0);
		CHK_RESULT(ret);
		ret = udCounterCtrl->setSelectedDevice(devInfo1884_1);
		CHK_RESULT(ret);
		ret = instantDoCtrl->setSelectedDevice(devInfo1730);
		CHK_RESULT(ret);
		ret = pwModulatorCtrl->setChannelStart(0);
		CHK_RESULT(ret);
		ret = pwModulatorCtrl->setChannelCount(4);
		CHK_RESULT(ret);
		poChannel = pwModulatorCtrl->getChannels();
		ret = udCounterCtrl->setChannelStart(0);
		CHK_RESULT(ret);
		ret = udCounterCtrl->setChannelCount(4);
		CHK_RESULT(ret);
		udChannel = udCounterCtrl->getChannels();

        // set initial duty, counting type
		for(int i=0; i<4; i++){
			pulseWidth[i] = {DUTY_CYCLE, 0.0};
			ret = poChannel->getItem(i).setPulseWidth(pulseWidth[i]);
			CHK_RESULT(ret);
			ret = udChannel->getItem(i).setCountingType(AbPhaseX4);
			CHK_RESULT(ret);
			instantDoCtrl->WriteBit(2, i, 1); //disable motor
			instantDoCtrl->WriteBit(2, i+4, direction[i]);
		}

        // enable
		ret= pwModulatorCtrl->setEnabled(true);
		CHK_RESULT(ret);
		ret= udCounterCtrl->setEnabled(true);
		CHK_RESULT(ret);
	}while(false);

	// If something wrong in this execution, print the error code on screen for tracking.
	if(BioFailed(ret)){
		wchar_t enumString[256];
		AdxEnumToString(L"ErrorCode", (int32)ret, 256, enumString);
		printf("Some error occurred. And the last error code is 0x%X. [%ls]\n", ret, enumString);
		return 0;
	}

    return 1;
} // daqInit()

int voltage2pulseWidth(double voltage[4], PulseWidth *pulseWidth, uint8 direction[4]){
	// std::cout << "voltage2pulseWidth" << std::endl;
	int i;
	double lo, vTemp[4];
	for(i=0; i<4; i++){
		if (voltage[i] == 0.0){
			(pulseWidth + i)->LoPeriod = 0.0;
			(pulseWidth + i)->HiPeriod = DUTY_CYCLE;
			continue;
		}
		if (voltage[i] > 0.0){
			vTemp[i] = voltage[i];
			direction[i] = (uint8)!(forwordDirection[i]);
		}
		else{
			vTemp[i] = 0.0 - voltage[i];
			direction[i] = (uint8)(forwordDirection[i]);
		}
		lo = ((vTemp[i]/24.0 + DEADZONE)*0.93 + 0.05)*DUTY_CYCLE; // 0~24(V) => 0.05~0.98(Duty)
		if (lo > DUTY_CYCLE) {lo = DUTY_CYCLE;}
		(pulseWidth + i)->LoPeriod = lo;
		(pulseWidth + i)->HiPeriod = DUTY_CYCLE - lo;
	}
	return 1;
}

int outputPwmDir(PulseWidth pulseWidth[4], Array<PoChannel> *poChannel, uint8 direction[4], InstantDoCtrl *instantDoCtrl){
	// std::cout << "outputPwmDir" << std::endl;
	int i;
	for(i=0; i<4; i++){
		instantDoCtrl->WriteBit(2, i+4, direction[i]);
		poChannel->getItem(i).setPulseWidth(pulseWidth[i]);
	}
	return 1;
}

int getTiresOmega(TIRES *tPtr){
	// std::cout << "getTiresOmega" << std::endl;
	int  i;
	int  ccnt[4]; // for Read(int32 count, int32 *data)
	int  temp;
	long dif;    // 8 Bytes  (cntNew, cntOld are 4 Bytes int)

	udCounterCtrl->Read(4, ccnt); //read QEP

	for(i=0; i<4; i++){
		tPtr->theTire[i].cntOld = tPtr->theTire[i].cntNew;
		tPtr->theTire[i].cntNew = ccnt[i];
		dif = (long)(tPtr->theTire[i].cntNew - tPtr->theTire[i].cntOld);
		//============= over float checking ==============
		if (dif > 100000){
			dif = dif - 4294967296; //2^32 = 4294967296
		}
		else if(dif < -100000){
			dif = dif + 4294967296;
		}
		// modify directions
		if (!forwordDirection[i]) {dif = -dif;}

		// 1rev => 10,000 cnt  ,   gear ratio => 10.203
		dataShift(tPtr->theTire[i].omega, 3);
		dataShift(tPtr->theTire[i].omegaLPF, 3);
		tPtr->theTire[i].omega[0] = dif*0.00615817436751895; //  1/10000*2*pi/0.01/10.203 = 0.00615817436751895
		LPfilter(tPtr->theTire[i].omega, tPtr->theTire[i].omegaLPF);
	}

	return 1;
}

inline void T2voltage(CONTROLLER_DATA ctrl[4], TIRES *t, double voltage[4]){
	// std::cout << "T2voltage" << std::endl;
	static const double K[4] = {6.2214, 5.6908, 6.7481, 6.4564};
	static const double B[4] = {3.2069, 2.8580, 3.4772, 3.3346};
	//(2)avg134: (K:6.4735 I:0.1331 B:3.3396)

	for(int i=0; i<4; i++){
		voltage[i] = ( ctrl[i].T + B[i]*(t->theTire[i].omegaLPF[0]) ) / K[i];

		// saturation check
		if (voltage[i] > 24.0){
			voltage[i] = 24.0;
			ctrl[i].T = 24.0*K[i] - B[i]*(t->theTire[i].omegaLPF[0]);
		}
		else if (voltage[i] < -24.0){
			voltage[i] = -24.0;
			ctrl[i].T = -24.0*K[i] - B[i]*(t->theTire[i].omegaLPF[0]);
		}
	}
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

int speedCtrl(double v[4], double spd[4], TIRES *t){
	static double err[4] = {0.0};
	static double I1[4] = {0.0};
	static double I2[4] = {0.0};

	for (int i=0; i<4; i++){
		err[i] = spd[i] - t->theTire[i].omegaLPF[0];
		I1[i] += err[i]*TS;
		I2[i] += I1[i]*TS;

		v[i] = 0.5*err[i] + 2.5*I1[i];// + 5.0*I2[i];
	}

	return 1;
}

void lowerLayerStateMachine(void){
	static double           t_last;

	if(Sensor){
		switch(lowerState){
			case OPENLOOP:
				for(int index=0; index<4; index++){
					controllerData[index].T = kktData.Fad[index]*TIRE_RADIUS;
				}
				//upperLayerData.Mlat = IZ*cg.dr[0] - upperLayerData.M;//((kktData.Fad[1]+kktData.Fad[3])*T_2 - (kktData.Fad[0]+kktData.Fad[2])*T_2);
			
				if(cga.VXLPF[0] > Vth){
					//init estimator velocities
					for(int index=0; index<4; index++){
						estimatorInit(estimatorData+index, index);
						(estimatorData+index)->omega_hat = tires.theTire[index].omegaLPF[0];
						(estimatorData+index)->omega_f_hat = tires.theTire[index].omegaLPF[0];
					}
					//switch state
					lowerState = EST_ON;
					t_last = sec;
				}
				break;

			case EST_ON:
				for(int index=0; index<4; index++){
					estimator(estimatorData+index, cgaPtr, tiresPtr, controllerData[index].T, sec, index);
					controllerData[index].T = kktData.Fad[index]*TIRE_RADIUS;
				}
				//upperLayerData.Mlat = IZ*cg.dr[0] - upperLayerData.M;//((kktData.Fad[1]+kktData.Fad[3])*T_2 - (kktData.Fad[0]+kktData.Fad[2])*T_2);

				if(cga.VXLPF[0] < Vth){	
					lowerState = OPENLOOP;
				}
				else if(sec - t_last > 0.2){
					lowerState = CTRL_ON;
					t_last = sec;
				}
				break;

			case CTRL_ON:
				for(int index=0; index<4; index++){
					estimator(estimatorData+index, cgaPtr, tiresPtr, controllerData[index].T, sec, index);
					torqueControl(controllerData+index, estimatorData+index, cgaPtr, tiresPtr, kktData.Fad[index], index);
					controllerData[index].T = kktData.Fad[index]*TIRE_RADIUS; // overwrite control
				}
				/*upperLayerData.Mlat = IZ*cg.dr[0] - ((estimatorData[1].mu_hat*estimatorData[1].Fz0 + estimatorData[3].mu_hat*estimatorData[3].Fz0)*T_2
				 - (estimatorData[0].mu_hat*estimatorData[0].Fz0 + estimatorData[2].mu_hat*estimatorData[2].Fz0)*T_2);*/

				if(cga.VXLPF[0] < Vth){
					lowerState = OPENLOOP;
				}
				else if(sec - t_last > 0.1){
					lowerState = CLOSELOOP;
				}
				break;

			case CLOSELOOP:
				for(int index=0; index<4; index++){
					estimator(estimatorData+index, cgaPtr, tiresPtr, controllerData[index].T, sec, index);
					torqueControl(controllerData+index, estimatorData+index, cgaPtr, tiresPtr, kktData.Fad[index], index);
				}
				/*upperLayerData.Mlat = IZ*cg.dr[0] - ((estimatorData[1].mu_hat*estimatorData[1].Fz0 + estimatorData[3].mu_hat*estimatorData[3].Fz0)*T_2
				 - (estimatorData[0].mu_hat*estimatorData[0].Fz0 + estimatorData[2].mu_hat*estimatorData[2].Fz0)*T_2);*/

				if(cga.VXLPF[0] < Vth){
					lowerState = OPENLOOP;
				}
				break;
		}
	}else{
		switch(lowerState){
			case OPENLOOP:
				for(int index=0; index<4; index++){
					controllerData[index].T = kktData.Fad[index]*TIRE_RADIUS;
				}
				//upperLayerData.Mlat = IZ*cg.dr[0] - upperLayerData.M;//((kktData.Fad[1]+kktData.Fad[3])*T_2 - (kktData.Fad[0]+kktData.Fad[2])*T_2);

				if(cg.VXLPF[0] > Vth){
					//init estimator velocities
					for(int index=0; index<4; index++){
						estimatorInit(estimatorData+index, index);
						(estimatorData+index)->omega_hat = tires.theTire[index].omegaLPF[0];
						(estimatorData+index)->omega_f_hat = tires.theTire[index].omegaLPF[0];
					}
					//switch state
					lowerState = EST_ON;
					t_last = sec;
				}
				break;

			case EST_ON:
				for(int index=0; index<4; index++){
					estimator(estimatorData+index, cgPtr, tiresPtr, controllerData[index].T, sec, index);
					controllerData[index].T = kktData.Fad[index]*TIRE_RADIUS;
				}
				//upperLayerData.Mlat = IZ*cg.dr[0] - upperLayerData.M;//((kktData.Fad[1]+kktData.Fad[3])*T_2 - (kktData.Fad[0]+kktData.Fad[2])*T_2);
				
				if(cg.VXLPF[0] < Vth){
					lowerState = OPENLOOP;
				}
				else if(sec - t_last > 0.2){
					lowerState = CTRL_ON;
					t_last = sec;
				}
				break;

			case CTRL_ON:
				for(int index=0; index<4; index++){
					estimator(estimatorData+index, cgPtr, tiresPtr, controllerData[index].T, sec, index);
					torqueControl(controllerData+index, estimatorData+index, cgPtr, tiresPtr, kktData.Fad[index], index);
					controllerData[index].T = kktData.Fad[index]*TIRE_RADIUS; // overwrite control
				}
				/*upperLayerData.Mlat = IZ*cg.dr[0] - ((estimatorData[1].mu_hat*estimatorData[1].Fz0 + estimatorData[3].mu_hat*estimatorData[3].Fz0)*T_2
				 - (estimatorData[0].mu_hat*estimatorData[0].Fz0 + estimatorData[2].mu_hat*estimatorData[2].Fz0)*T_2);*/

				if(cg.VXLPF[0] < Vth){
					lowerState = OPENLOOP;
				}
				else if(sec - t_last > 0.1){
					lowerState = CLOSELOOP;
				}
				break;

			case CLOSELOOP:
				for(int index=0; index<4; index++){
					estimator(estimatorData+index, cgPtr, tiresPtr, controllerData[index].T, sec, index);
					torqueControl(controllerData+index, estimatorData+index, cgPtr, tiresPtr, kktData.Fad[index], index);
				}
				/*upperLayerData.Mlat = IZ*cg.dr[0] - ((estimatorData[1].mu_hat*estimatorData[1].Fz0 + estimatorData[3].mu_hat*estimatorData[3].Fz0)*T_2
			 	- (estimatorData[0].mu_hat*estimatorData[0].Fz0 + estimatorData[2].mu_hat*estimatorData[2].Fz0)*T_2);*/

				if(cg.VXLPF[0] < Vth){
					lowerState = OPENLOOP;
				}
				break;
		}
	}
	
} // void lowerLayerStateMachine(void)

void assignForce(double Fad[4], double assignVal, double sec, double t1, double t2){
	if (sec<t1){
		for(int i=0; i<4; i++) {Fad[i] = assignVal*sec/t1;}
	}
	else if(sec<t2){
		for(int i=0; i<4; i++) {Fad[i] = assignVal;}
	}
	else if(sec<t2*2.0){
		for(int i=0; i<4; i++) {Fad[i] = -assignVal;}
	}
	else{
		for(int i=0; i<4; i++) {Fad[i] = 0.0;}
	}
}

/*
[[[[[ motor transfer function ]]]]]
	w/v = K/(IS+B)  ( volt => rad/s(wheel) )

	(1) K:6.2214  I:0.1277  B:3.2069
	(2) K:5.1695  I:0.1051  B:2.5962
		((2) K:5.6908  I:0.1157  B:2.8580)
		avg134: (K:6.4735 I:0.1331 B:3.3396)
	(3) K:6.7481  I:0.1383  B:3.4772
	(4) K:6.4564  I:0.1332  B:3.3346
*/

void read_K(double k_gain[][8],int r_len){
    int i,j;
    //double bb[50][8];

    FILE* fp;
		memset(fileName,0,80);
		strcpy(fileName, FILE_DIR);
		strcat(fileName,"../../config/KK.txt");
    fp = fopen(fileName,"r");

    if(fp==NULL){
        printf("error");
    }

    for(i=0;i<r_len;i++){
        for(j=0;j<8;j++){

            fscanf(fp,"%lf",&k_gain[i][j]);
        }
        fscanf(fp,"\n");

    }

    fclose(fp);
}

void read_dK(double dk_gain[][8],int r_len){
    int i,j;
    //double bb[50][8];

    FILE* dfp;
	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName,"../../config/dKK.txt");
    dfp = fopen(fileName,"r");

    if(dfp==NULL){
        printf("error");
    }

    for(i=0;i<r_len;i++){
        for(j=0;j<8;j++){

            fscanf(dfp,"%lf",&dk_gain[i][j]);
        }
        fscanf(dfp,"\n");

    }

    fclose(dfp);
}

void read_Pe(double Pe[][6],int r_len){
    int i,j;
    //double bb[50][8];

    FILE* fpe;
	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName,"../../config/Pe.txt");
    fpe = fopen(fileName,"r");

    if(fpe==NULL){
        printf("error");
    }

    for(i=0;i<r_len;i++){
        for(j=0;j<6;j++){

            fscanf(fpe,"%lf",&Pe[i][j]);
        }
        fscanf(fpe,"\n");

    }

    fclose(fpe);
}

void read_b(double b[],int r_len){
    int i,j;
    //double bb[50][8];

    FILE* fb;
	memset(fileName,0,80);
	strcpy(fileName, FILE_DIR);
	strcat(fileName,"../../config/b.txt");
    fb = fopen(fileName,"r");

    if(fb==NULL){
        printf("error");
    }

    for(i=0;i<r_len;i++){


        fscanf(fb,"%lf",&b[i]);
        fscanf(fb,"\n");

    }

    fclose(fb);
}

void write_KK(FILE *KKfPtr,double k_gain[][8],int r_len){
	int i,j;

    for(i=0;i<r_len;i++){
        for(j=0;j<8;j++){
			fprintf(KKfPtr,"%f ",k_gain[i][j]);
            
        }
        fprintf(KKfPtr,"\n");
    }

}
void write_M1(FILE *M1fPtr,double M1[][9],int time_len){
	int i,j;

    for(i=0;i<time_len;i++){
        for(j=0;j<9;j++){
			fprintf(M1fPtr,"%f ",M1[i][j]);
            
        }
        fprintf(M1fPtr,"\n");
    }

}
void write_cg(FILE *cgfPtr,double cg1[][21],int time_len){
	int i,j;

    for(i=0;i<time_len;i++){
        for(j=0;j<21;j++){
			fprintf(cgfPtr,"%f ",cg1[i][j]);
            
        }
        fprintf(cgfPtr,"\n");
    }

}
void write_ul(FILE *ulfPtr,double ul1[][24],int time_len){
	int i,j;

    for(i=0;i<time_len;i++){
        for(j=0;j<24;j++){
			fprintf(ulfPtr,"%f ",ul1[i][j]);
            
        }
        fprintf(ulfPtr,"\n");
    }

}
void write_F(FILE *FfPtr,double F1[][15],int time_len){
	int i,j;

    for(i=0;i<time_len;i++){
        for(j=0;j<15;j++){
			fprintf(FfPtr,"%f ",F1[i][j]);
            
        }
        fprintf(FfPtr,"\n");
    }

}
void write_tck(FILE *tckfPtr,double tck1[][7],int time_len){
	int i,j;

    for(i=0;i<time_len;i++){
        for(j=0;j<7;j++){
			fprintf(tckfPtr,"%f ",tck1[i][j]);
            
        }
        fprintf(tckfPtr,"\n");
    }

}
void write_amcl(FILE *amclfPtr,double amcl1[][14],int time_len){
	int i,j;

    for(i=0;i<time_len;i++){
        for(j=0;j<14;j++){
			fprintf(amclfPtr,"%f ",amcl1[i][j]);
            
        }
        fprintf(amclfPtr,"\n");
    }

}
void write_odom(FILE *odomfPtr,double odom1[][3],int time_len){
	int i,j;

    for(i=0;i<time_len;i++){
        for(j=0;j<3;j++){
			fprintf(odomfPtr,"%f ",odom1[i][j]);
            
        }
        fprintf(odomfPtr,"\n");
    }

}
void write_KKplus(FILE *KKplusfPtr,double k_gain_plus[][8],int r_len){
	int i,j;

    for(i=0;i<r_len;i++){
        for(j=0;j<8;j++){
			fprintf(KKplusfPtr,"%f ",k_gain_plus[i][j]);
            
        }
        fprintf(KKplusfPtr,"\n");
    }

}

void write_dKKplus(FILE *dKKplusfPtr,double dk_gain_plus[][8],int r_len){
	int i,j;

    for(i=0;i<r_len;i++){
        for(j=0;j<8;j++){
			fprintf(dKKplusfPtr,"%f ",dk_gain_plus[i][j]);
            
        }
        fprintf(dKKplusfPtr,"\n");
    }

}