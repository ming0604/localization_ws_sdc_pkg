#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sstream>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Joy.h>
#include <message_filters/subscriber.h>

#include "/opt/advantech/examples/C++_Console/inc/compatibility.h"
#include "/opt/advantech/inc/bdaqctrl.h"

#include "signalProcessing.h"

#define  PI 3.1415926

#define  deviceDescription1884_0  L"PCIE-1884,BID#0"
#define  deviceDescription1884_1  L"PCIE-1884,BID#1"
#define  deviceDescription1730    L"PCI-1730,BID#0"
#define  TS                       0.01  // sampling time (sec)
#define  DUTY_CYCLE               0.002 // 500 Hz PWM
#define  DEADZONE                 0.005 // 0.5% deadzone

using namespace Automation::BDaq;

// =============== functions ===================
char idle(void);
int emergencyStop();
int daqInit(void); // initialization of PWM, QEP, DO
int voltage2pulseWidth(double voltage[4], PulseWidth *pulseWidth, uint8 direction[4]);
int outputPwmDir(PulseWidth pulseWidth[4], Array<PoChannel> *poChannel, uint8 direction[4], InstantDoCtrl *instantDoCtrl);

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
			lwheel = 0;
			rwheel = 0;
			lf = 0;
			rf = 0;
			tiresPtr = &tires;
		}

		void timer_start(){ 	
            timer1 = n.createTimer(ros::Duration(0.01), &Wodom::callback, this);
			// timer = n.createTimer(ros::Duration(0.01), &Tracker::timerCallback, this);
        }

        void callback(const ros::TimerEvent& e){
            std_msgs::Int32MultiArray wheel;
            wheel.data.clear();
            getTiresOmega(tiresPtr);
            //msg_lwheel.data = lwheel;
            wheel.data.push_back(lwheel);
            wheel.data.push_back(rwheel);
			wheel.data.push_back(lf);
			wheel.data.push_back(rf);
            chatter_pub.publish(wheel);
        }

        int getTiresOmega(TIRES *tPtr){
			int  i;
			int  cnt[4] = {0}; // for Read(int32 count, int32 *data)
			int  temp;
			long dif;    // 8 Bytes  (cntNew, cntOld are 4 Bytes int)

			udCounterCtrl->Read(4, cnt); //read QEP

			for(i=0; i<4; ++i){
				//tPtr->theTire[i].cntOld = tPtr->theTire[i].cntNew;
				tPtr->theTire[i].cntNew = cnt[i];
				/*
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
				*/

			}
			lwheel = tPtr->theTire[0].cntNew;
			rwheel = tPtr->theTire[1].cntNew;
			lf = tPtr->theTire[2].cntNew;
			rf = tPtr->theTire[3].cntNew;
			// printf("%d %d %d %d\n", cnt[0], cnt[1], cnt[2], cnt[3]);
			// printf("%d %d %d %d\n", tPtr->theTire[0].cntNew, tPtr->theTire[1].cntNew, tPtr->theTire[2].cntNew, tPtr->theTire[3].cntNew);
			
			// printf("\n");
				
			return 1;
		}
        
        // void timer_start(){ 	
        //     timer1 = n.createTimer(ros::Duration(0.01), &Wodom::callback, this);
        // }

    private:
        int lwheel, rwheel;
		int lf, rf;

        TIRES  tires, *tiresPtr;
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("wheel", 100);

        ros::Timer timer1;
        
};


int main(int argc, char **argv){
	
	ros::init(argc, argv, "Remote");
	Joy remoter;
	Wodom talker;
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

		ROS_INFO("Initializing PCIE-1884,BID#1 for Counter...");
		ret = udCounterCtrl->setSelectedDevice(devInfo1884_1);
		CHK_RESULT(ret);
		ROS_INFO("Initializing PCIE-1884,BID#0 for PWM...");
		ret = pwModulatorCtrl->setSelectedDevice(devInfo1884_0);
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
	int i;
	for(i=0; i<4; i++){
		instantDoCtrl->WriteBit(2, i+4, direction[i]);
		poChannel->getItem(i).setPulseWidth(pulseWidth[i]);
	}
	return 1;
}


