#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <math.h>
#include <unistd.h>

#include "/opt/advantech/examples/C++_Console/inc/compatibility.h"
#include "/opt/advantech/inc/bdaqctrl.h"
#include "signalProcessing.h"

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
int getTiresOmega(TIRES *tPtr);
// ================ globals ====================

//TIRES            tires, *tiresPtr;
ErrorCode        ret = Success;
PwModulatorCtrl  *pwModulatorCtrl = PwModulatorCtrl::Create();
UdCounterCtrl    *udCounterCtrl = UdCounterCtrl::Create();
InstantDoCtrl    *instantDoCtrl = InstantDoCtrl::Create();
Array<PoChannel> *poChannel;
Array<UdChannel> *udChannel;

const bool forwordDirection[4] = {true,false,true,false};
PulseWidth pulseWidth[4];
uint8      direction[4] = {1,0,1,0};
double     speed[4] = {0.0,0.0,0.0,0.0};

class Node{
	public:
		Node(){
		}

        void callback(const ros::TimerEvent&){
            std_msgs::Int32MultiArray wheel;
            wheel.data.clear();
            getTiresOmega(tiresPtr);
            //msg_lwheel.data = lwheel;
            wheel.data.push_back(lwheel);
            wheel.data.push_back(rwheel);
            chatter_pub.publish(wheel);
			
        }

        int getTiresOmega(TIRES *tPtr){
			int  i;
			int  cnt[4]; // for Read(int32 count, int32 *data)
			int  temp;
			long dif;    // 8 Bytes  (cntNew, cntOld are 4 Bytes int)

			udCounterCtrl->Read(4, cnt); //read QEP

			for(i=0; i<4; i++){
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
			lwheel = tPtr->theTire[0].cntNew;
			rwheel = tPtr->theTire[1].cntNew;
			//printf("\n");
				
			return 1;
		}
        
        void timer_start(){ 	
            timer1 = n.createTimer(ros::Duration(0.01), &Node::callback, this);
        }

    private:
        int lwheel = 0;
		int rwheel = 0;
        TIRES            tires, *tiresPtr;
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("wheel", 100);

        ros::Timer timer1;
        
        
        
};


int main(int argc, char **argv){
	char c;
	int a;
	
    ros::init(argc, argv, "talker");
    printf("build node\n");
    Node talk;
	talk.timer_start();
	
	// // ======================== DAQ init ========================
	// printf("start daq init\n");
    // if(daqInit() == 0){
    //     printf("error: daqInit()\n");
    //     return 0;
    // }
    // printf("end\n");
    
    ros::spin();
    return 0;
    
} // main()


//====================================================
//                      idle()
//====================================================
char idle(void)
{
	char c;
	
	while(true){
		c = getchar();
		if (c =='\n') break;
	}
	return c;
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

//====================================================
//                    getTiresOmega
//====================================================

int getTiresOmega(TIRES *tPtr)
{
	int  i;
	int  cnt[4]; // for Read(int32 count, int32 *data)
	int  temp;
	long dif;    // 8 Bytes  (cntNew, cntOld are 4 Bytes int)

	udCounterCtrl->Read(4, cnt); //read QEP

	for(i=0; i<4; i++){
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
		printf("%d %ld\t", i, (long)(tPtr->theTire[i].cntNew));

	}
	printf("\n");
	
	return (int)tPtr->theTire[i].cntNew;
}
