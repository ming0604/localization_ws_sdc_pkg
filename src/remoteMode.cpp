#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <ros/ros.h>
#include <sstream>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>
#include <message_filters/subscriber.h>

#include "/opt/advantech/examples/C++_Console/inc/compatibility.h"
#include "/opt/advantech/inc/bdaqctrl.h"

#include "signalProcessing.h"

#include "sdc/FourWheelsData.h"

#define  PI 3.1415926

#define  deviceDescription1884_0  L"PCIE-1884,BID#0"
#define  deviceDescription1884_1  L"PCIE-1884,BID#1"
#define  deviceDescription1730    L"PCI-1730,BID#0"
#define  TS                       0.01  // sampling time (sec)
#define  DUTY_CYCLE               0.002 // 500 Hz PWM
#define  DEADZONE                 0.005 // 0.5% deadzone

#define MAX_TIRE_ANGULAR_VELOCITY_THRESHOLD 25.0  // maximum angular velocity threshold(rad/s) for rushing detection
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

using namespace Automation::BDaq;

// =============== functions ===================
char idle(void);
int emergencyStop();
int daqInit(void); // initialization of PWM, QEP, DO
int voltage2pulseWidth(double voltage[4], PulseWidth *pulseWidth, uint8 direction[4]);
int outputPwmDir(PulseWidth pulseWidth[4], Array<PoChannel> *poChannel, uint8 direction[4], InstantDoCtrl *instantDoCtrl);
void core_dump_handler(int signum);
// ================ globals ====================
ErrorCode        ret = Success;
PwModulatorCtrl  *pwModulatorCtrl = PwModulatorCtrl::Create();
UdCounterCtrl    *udCounterCtrl = UdCounterCtrl::Create();
InstantDoCtrl    *instantDoCtrl = InstantDoCtrl::Create();
Array<PoChannel> *poChannel;
Array<UdChannel> *udChannel;

const bool forwordDirection[4] = {true,false,true,false};	// 0: CW  1: CCW
PulseWidth pulseWidth[4];
uint8      direction[4] = {1,0,1,0};
double     voltage[4] = {0.0,0.0,0.0,0.0};	//(lf, rf, lb, rb)
double 	   v, a;
int 	   speed_g, speed_d;
int        istop = 0;
int 	   consecutive_cnt_error = 0;
class Joy{
	public:
		Joy(){
			joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, &Joy::joyCallback, this);
		}
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
			// ROS_INFO("stamp: %f", joy->header.stamp.toSec());
			// ROS_INFO("axes: %f %f %f %f %f %f", joy->axes[0], joy->axes[1], joy->axes[2], joy->axes[3], joy->axes[4], joy->axes[5]);
			// ROS_INFO("buttons: %d %d %d %d %d %d", joy->buttons[0], joy->buttons[1], joy->buttons[2], joy->buttons[3], joy->buttons[4], joy->buttons[5]);
			// ROS_INFO("buttons: %d %d %d %d %d %d", joy->buttons[6], joy->buttons[7], joy->buttons[8], joy->buttons[9], joy->buttons[10], joy->buttons[11]);
		  
			//button B
			if(joy->buttons[2] == 1){
				ROS_INFO("Emergency Stop");
			  	v = 0;
			    a = 0;
			    speed_g = 1;
			    speed_d = 1;
			  	for(int i=0; i<4; i++){
					instantDoCtrl->WriteBit(2, i, 1); //unable motor
				}
				exit(0);
			//button LT
			}else if(joy->buttons[6] == 1){
			  	ROS_INFO("Diff low");
			  	if(speed_d > 1)
			  		speed_d -= 1;
			//button RT
			}else if(joy->buttons[7] == 1){
			  	ROS_INFO("Diff high");
			  	if(speed_d < 14)
			  		speed_d += 1;
			//button LB
			}else if(joy->buttons[4] == 1){
				ROS_INFO("Slow down");
			  	if(speed_g > 1)
			  		speed_g -= 1;
			//button RB
			}else if(joy->buttons[5] == 1){
				ROS_INFO("Speed up");
			  	if(speed_g < 10)
			  		speed_g += 1;
			}
			// //axes 1: front & back
			// else if(joy->axes[1]){
			// 	v = (joy->axes[1]*speed_g)/2.;
			// //axes 2: left & right
			// }else if(joy->axes[2] > 0){
			// 	a = (joy->axes[2]*speed_d)/2.;
			// }else if(joy->axes[2] < 0){
			// 	a = -(joy->axes[2]*speed_d)/2.;
			// }
			// printf("%f %f\n", v, a);
			//axes 1: front & back
			v = (joy->axes[1]*speed_g)/2.;
			//axes 2: left & right
			if(joy->axes[2] > 0){
				a = (joy->axes[2]*speed_d)/2.;
				// if(joy->axes[2] > 0){
				// 	voltage[0] = -v - a;
				// 	voltage[1] = -v + a;
				// 	voltage[2] = -v - a;
				// 	voltage[3] = -v + a;
				// }else if(joy->axes[2] < 0){
				// 	voltage[0] = -v + a;
				// 	voltage[1] = -v - a;
				// 	voltage[2] = -v + a;
				// 	voltage[3] = -v - a;
				// }else{
				// 	voltage[0] = -v;
				// 	voltage[1] = -v;
				// 	voltage[2] = -v;
				// 	voltage[3] = -v;
				// }
			}
			else{
				a = -(joy->axes[2]*speed_d)/2.;	
				// if(joy->axes[2] < 0){
				// 	voltage[0] = -v - a;
				// 	voltage[1] = -v + a;
				// 	voltage[2] = -v - a;
				// 	voltage[3] = -v + a;
				// }else if(joy->axes[2] > 0){
				// 	voltage[0] = -v + a;
				// 	voltage[1] = -v - a;
				// 	voltage[2] = -v + a;
				// 	voltage[3] = -v - a;
				// }else{
				// 	voltage[0] = -v;
				// 	voltage[1] = -v;
				// 	voltage[2] = -v;
				// 	voltage[3] = -v;
				// }
			}

			if(v != 0 || a != 0){
				// printf("Go\n");
			  	for(int i=0; i<4; i++){
					instantDoCtrl->WriteBit(2, i, 0); //enable motor
			  	}
			}else{
				// printf("No\n");
				emergencyStop();
				// for(int i=0; i<4; i++){
				// 	instantDoCtrl->WriteBit(2, i, 1); //unable motor
				// }
			}
			//buttons 4: left & right in a circle
			// if(joy->axes[4] == 1 && speed_g == 0){
			// 	ROS_INFO("Left");
			//   	voltage[0] = -3;
			// 	voltage[1] = 3;
			// 	voltage[2] = -3;
			// 	voltage[3] = 3;
			// }else if(joy->axes[4] == -1 && speed_g == 0){
			// 	ROS_INFO("Right");
			// 	voltage[0] = 3;
			// 	voltage[1] = -3;
			// 	voltage[2] = 3;
			// 	voltage[3] = -3;
			// }else

			if(joy->axes[2] > 0){
			  	voltage[0] = v - a;
				voltage[1] = v + a;
				voltage[2] = v - a;
				voltage[3] = v + a;
			}else if(joy->axes[2] < 0){
				voltage[0] = v + a;
				voltage[1] = v - a;
				voltage[2] = v + a;
				voltage[3] = v - a;
			}else{
				voltage[0] = v;
				voltage[1] = v;
				voltage[2] = v;
				voltage[3] = v;
			}
			voltage2pulseWidth(voltage, pulseWidth, direction);
	        outputPwmDir(pulseWidth, poChannel, direction, instantDoCtrl);
			printf("(lf, rf, lb, rb, g, d) = %f %f %f %f %d %d\n", voltage[0], voltage[1], voltage[2], voltage[3], speed_g, speed_d);
			printf("(lo[0], lo[1], lo[2], lo[3]) = %f %f %f %f\n", pulseWidth[0].LoPeriod, pulseWidth[1].LoPeriod, pulseWidth[2].LoPeriod, pulseWidth[3].LoPeriod);
		}
	private:
		ros::NodeHandle n;
		ros::Subscriber joy_sub;
};

class Wodom{
	public:
		Wodom(){

			tiresPtr = &tires;
		}

		void timer_start(){ 	
            timer1 = n.createTimer(ros::Duration(0.01), &Wodom::callback, this);
			// timer = n.createTimer(ros::Duration(0.01), &Tracker::timerCallback, this);
        }

        void callback(const ros::TimerEvent& e){
			/*           
			std_msgs::Int32MultiArray wheel;
            wheel.data.clear();
            getTiresOmega(tiresPtr);
            //msg_lwheel.data = lwheel;
            wheel.data.push_back(lwheel);
            wheel.data.push_back(rwheel);
			wheel.data.push_back(lf);
			wheel.data.push_back(rf);
            chatter_pub.publish(wheel); 
			*/

			sdc::FourWheelsData wheels;
			int getTiresOmega_result;
			// record time stamp
			wheels.header.stamp = ros::Time::now();
	
			getTiresOmega_result = getTiresOmega(tiresPtr);

			// record counts and angular velocities of four tires
			wheels.counts[0] = tires.theTire[0].cntNew;
			wheels.counts[1] = tires.theTire[1].cntNew;
			wheels.counts[2] = tires.theTire[2].cntNew;
			wheels.counts[3] = tires.theTire[3].cntNew;
			wheels.angular_velocities[0] = tires.theTire[0].omegaLPF[0];
			wheels.angular_velocities[1] = tires.theTire[1].omegaLPF[0];
			wheels.angular_velocities[2] = tires.theTire[2].omegaLPF[0];
			wheels.angular_velocities[3] = tires.theTire[3].omegaLPF[0];
			// publish the message
			wheels_data_pub.publish(wheels);

			// if getTiresOmega returns -1, it means some error occurs and emergency stop has been activated
			if(getTiresOmega_result == -1)
			{
				// stop the timer to prevent further reading
				timer1.stop();
				ROS_ERROR("closing the timer in Wodom::callback and then shut down the node.");
				exit(EXIT_FAILURE);
			}
        }

        int getTiresOmega(TIRES *tPtr){
			int  i;
			int  cnt[4] = {0}; // for Read(int32 count, int32 *data)
			int  temp;
			long dif;    // 8 Bytes  (cntNew, cntOld are 4 Bytes int)
			
			ErrorCode ret_encoder = Success;
			ret_encoder = udCounterCtrl->Read(4, cnt); //read QEP
			// If something wrong in this execution, print the error code on screen for tracking.
			if(BioFailed(ret_encoder)){
				wchar_t enumString[256];
				AdxEnumToString(L"ErrorCode", (int32)ret_encoder, 256, enumString);
				ROS_ERROR("Some error occurred in Wodom::getTiresOmega. And the last error code is 0x%X. [%ls]\n", ret_encoder, enumString);
			}

			for(i=0; i<4; ++i){
				tPtr->theTire[i].cntOld = tPtr->theTire[i].cntNew;
				tPtr->theTire[i].cntNew = cnt[i];
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
				//printf("%d %ld\t", i, (long)(tPtr->theTire[i].cntNew));
				
			}
			//printf("omega:%.3f, %.3f, %.3f, %.3f\n", tPtr->theTire[0].omega[0], tPtr->theTire[1].omega[0], tPtr->theTire[2].omega[0], tPtr->theTire[3].omega[0]);
			// ========== check tire safety ==========
			if(tire_safety_check(tPtr) == -1)
			{
				// not safe, emergency stop
				ROS_ERROR("activating emergency stop!");
				istop = emergencyStop();
				return -1;
			}
			else
			{
				// safe, return 1
				return 1;
			}

			// printf("%d %d %d %d\n", cnt[0], cnt[1], cnt[2], cnt[3]);
			// printf("%d %d %d %d\n", tPtr->theTire[0].cntNew, tPtr->theTire[1].cntNew, tPtr->theTire[2].cntNew, tPtr->theTire[3].cntNew);
			
			// printf("\n");
				
			// return 1;
		}

		int tire_safety_check(TIRES *tPtr)
		{
			bool ang_vel_too_high = false;
			bool cnt_error = false;
			bool cnt_all_negative_one = true;

			// 1. check if any tire's angular velocity is over the threshold
			for(int i=0; i<4; i++)
			{
				if(fabs(tPtr->theTire[i].omegaLPF[0]) > MAX_TIRE_ANGULAR_VELOCITY_THRESHOLD)
				{
					ang_vel_too_high = true;
					ROS_ERROR("Tire %d angular velocity is %f (rad/s), which is over the threshold %f (rad/s)!", i, tPtr->theTire[i].omegaLPF[0], MAX_TIRE_ANGULAR_VELOCITY_THRESHOLD);
				}
			}

			// 2. check if all tires' encoder counts are -1
			for(int i=0; i<4; i++)
			{
				if(tPtr->theTire[i].cntNew != -1)
				{
					cnt_all_negative_one = false;
					break;
				}
			}
			// if all tires' encoder counts are -1, increase the consecutive count error variable
			if(cnt_all_negative_one)
			{	
				consecutive_cnt_error++;
				ROS_WARN("Warning: All encoder counts are -1 (consecutive count: %d/3)", consecutive_cnt_error);
				if(consecutive_cnt_error >= 3) // if reach 3 consecutive times, set the count error flag
				{
					cnt_error = true;
					ROS_ERROR("All encoder counts are -1 for 3 consecutive times!");
				}
			}
			else // if not all -1, reset the consecutive count error variable
			{
				consecutive_cnt_error = 0; 
			}

			// 3. if any of the two error flags is true, return -1 (not safe); else return 1 (safe)
			if(ang_vel_too_high || cnt_error)
			{
				ROS_ERROR("Tire is not safe!");
				return -1;
			}
			else
			{
				return 1;
			}
		}
        
        // void timer_start(){ 	
        //     timer1 = n.createTimer(ros::Duration(0.01), &Wodom::callback, this);
        // }

    private:
        TIRES  tires, *tiresPtr;
        ros::NodeHandle n;
        // ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("wheel", 100);
		ros::Publisher wheels_data_pub = n.advertise<sdc::FourWheelsData>("wheel", 100);
        ros::Timer timer1;
        
};


int main(int argc, char **argv){
	
	signal(SIGSEGV, core_dump_handler);
	signal(SIGABRT, core_dump_handler);
	signal(SIGFPE, core_dump_handler);
	signal(SIGILL, core_dump_handler);
	signal(SIGBUS, core_dump_handler);

	ros::init(argc, argv, "Remote");
	Joy remoter;
	Wodom talker;
	// sleep 0.5 sec to wait subscriber and publisher to be ready
	ros::Duration(0.5).sleep();
	talker.timer_start();
	// char   c;  // use to catch emergencyStop
 //    double v;

	// if (argc!=2){
	// 	printf("argument not correct\n");
	// 	return 1;
	// }

    // v = atof(argv[1]);
 //    if (v<0.0){
 //        v=-v;
 //    }
	// if (v>5.0){
	// 	v=5.0;
	// }

	// ======================== DAQ init ========================
    if(daqInit() == 0){
        printf("error: daqInit()\n");
        return 0;
    }
    ROS_INFO("DAQ initial");
    v = 0;
    a = 0;
    speed_g = 1;
    speed_d = 1;
    ros::spin();

  //   while(true){
  //       // c = idle();

		// switch(c){
		// 	case '2':
		// 		for(int i=0; i<4; i++) {voltage[i]=v;}
		// 		//printf("voltage set: %f\n",voltage[0]);
		// 		break;
		// 	case '6':
		// 		for(int i=0; i<4; i++) {voltage[i]=v;}
		// 		voltage[1]=-voltage[1];
		// 		voltage[3]=-voltage[3];
		// 		//printf("voltage set: %f\n",voltage[0]);
		// 		break;
		// 	case '4':
		// 		for(int i=0; i<4; i++) {voltage[i]=v;}
		// 		voltage[0]=-voltage[0];
		// 		voltage[2]=-voltage[2];
		// 		//printf("voltage set: %f\n",voltage[0]);
		// 		break;
		// 	case '8':
		// 		for(int i=0; i<4; i++) {voltage[i]=-v;}
		// 		//printf("voltage set: %f\n",voltage[0]);
		// 		break;
		// 	case '9':
		// 		voltage[0] = 2.0*v;
		// 		voltage[1] = 0.5*v;
		// 		voltage[2] = 2.0*v;
		// 		voltage[3] = 0.5*v;
		// 		break;
		// 	case '7':
		// 		voltage[0] = 0.5*v;
		// 		voltage[1] = 2.0*v;
		// 		voltage[2] = 0.5*v;
		// 		voltage[3] = 2.0*v;
		// 		break;			
		// 	default:
		// 		for(int i=0; i<4; i++) {voltage[i]=0.0;}
		// 		printf("voltage set: %f\n",voltage[0]);
		// 		break;
		// }

		// for(int i=0; i<4; i++) {voltage[i]=v;}
        // voltage2pulseWidth(voltage, pulseWidth, direction);
        // outputPwmDir(pulseWidth, poChannel, direction, instantDoCtrl);
  //       printf("(Enter to stop.)\n");

  //       c = getchar();
  //       if ( c=='\n' || (c>'a' && c<'z') || (c>'A' && c<'Z') || (c>'0' && c<'9')){
  //           emergencyStop();
  //       }
  //   }

    return 1;
} // main()


//====================================================
//                      idle()
//====================================================
char idle(void)
{
	char c;
	
	while(true){
		printf("(8:forward  6:right turn  4:left turn  2:backward)\n");
		scanf("%c",&c);
		getchar();
		if (c>='1' && c<='9') break;
	}

	for(int i=0; i<4; i++){
		instantDoCtrl->WriteBit(2, i, 0); //enable motor
	}

	return c;
}


//====================================================
//             emergencyStop(timerPtr)
//====================================================
int emergencyStop()
{
    for(int i=0;i<4;i++){
        pulseWidth[i].LoPeriod = 0.0;
        pulseWidth[i].HiPeriod = DUTY_CYCLE;
    }

	// stop motor
	for(int i=0; i<4; i++){
		instantDoCtrl->WriteBit(2, i, 1); //disable motor
		poChannel->getItem(i).setPulseWidth(pulseWidth[i]);
	}

	printf("stopped.\n");
	return 1;
}


//====================================================
//                     daqInit()
//====================================================
int daqInit(void)
{
    for(int i=0;i<4;i++){
        pulseWidth[i].LoPeriod = 0.0;
        pulseWidth[i].HiPeriod = DUTY_CYCLE;
    }

    do{
		DeviceInformation devInfo1884_0(deviceDescription1884_0);
		DeviceInformation devInfo1884_1(deviceDescription1884_1);
		DeviceInformation devInfo1730(deviceDescription1730);
		ROS_INFO("Initializing PCIE-1884,BID#0 for PWM...");
		ret = pwModulatorCtrl->setSelectedDevice(devInfo1884_0);
		CHK_RESULT(ret);
		ROS_INFO("Initializing PCIE-1884,BID#1 for Counter...");
		ret = udCounterCtrl->setSelectedDevice(devInfo1884_1);
		CHK_RESULT(ret);
		ROS_INFO("Initializing PCI-1730,BID#0 for Digital I/O...");
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


//====================================================
//                voltage2pulseWidth
//====================================================
int voltage2pulseWidth(double voltage[4], PulseWidth *pulseWidth, uint8 direction[4])
{
	int i;
	double lo, vTemp[4];
	for(i=0; i<4; i++){
		if (voltage[i] == 0.0){
			(pulseWidth + i)->LoPeriod = 0.0;
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
		// printf("%d ", direction[i]);
	}
	// printf("\n");
	return 1;
}


//====================================================
//                    outputPwmDir
//====================================================
int outputPwmDir(PulseWidth pulseWidth[4], Array<PoChannel> *poChannel, uint8 direction[4], InstantDoCtrl *instantDoCtrl)
{	
	ErrorCode ret_dir = Success;
    ErrorCode ret_pwm = Success;
	int i;
	for(i=0; i<4; i++){
		ret_dir = instantDoCtrl->WriteBit(2, i+4, direction[i]);
			
		if(BioFailed(ret_dir)){
			wchar_t enumString[256];
			AdxEnumToString(L"ErrorCode", (int32)ret_dir, 256, enumString);
			ROS_ERROR("Some error occurred in outputPwmDir's direction control of motor %d. And the last error code is 0x%X. [%ls]\n", i, ret_dir, enumString);
		}

		ret_pwm = poChannel->getItem(i).setPulseWidth(pulseWidth[i]);
		if(BioFailed(ret_pwm)){
			wchar_t enumString[256];
			AdxEnumToString(L"ErrorCode", (int32)ret_pwm, 256, enumString);
			ROS_ERROR("Some error occurred in outputPwmDir's PWM output of motor %d. And the last error code is 0x%X. [%ls]\n", i, ret_pwm, enumString);
		}
	}
	return 1;
}

//====================================================
//                     core_dump_handler()
//====================================================
void core_dump_handler(int signum){
	FILE *log_file;
	char buffer[256];

	printf(ANSI_COLOR_RED "[Core dumped!!]\n" ANSI_COLOR_RESET);

	istop = emergencyStop();
	for(int index=0; index<4; index++){
		voltage[index] = 0.0;
	}

	log_file = fopen("core_dump.log", "a");
	if(log_file == NULL){
		perror("Failed to open log file");
		exit(EXIT_FAILURE);
	}

	snprintf(buffer, sizeof(buffer), "Core dumped! Signal number: %d\n", signum);
	fputs(buffer, log_file);

	fclose(log_file);

	exit(EXIT_FAILURE);
}
