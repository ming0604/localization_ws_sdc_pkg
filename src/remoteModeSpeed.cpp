#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "/opt/advantech/examples/C++_Console/inc/compatibility.h"
#include "/opt/advantech/inc/bdaqctrl.h"

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
int speed2pulseWidth(double speed[4], PulseWidth *pulseWidth, uint8 direction[4]);
int outputPwmDir(PulseWidth pulseWidth[4], Array<PoChannel> *poChannel, uint8 direction[4], InstantDoCtrl *instantDoCtrl);

// ================ globals ====================
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


int main(int argc, char **argv)
{
	char   c;  // use to catch emergencyStop
    double v;

	if (argc!=2){
		printf("argument not correct\n");
		return 1;
	}

    v = atof(argv[1]);
    if (v<0.0){
        v=-v;
    }
	if (v>5.0){
		v=5.0;
	}

	// ======================== DAQ init ========================
    if(daqInit() == 0){
        printf("error: daqInit()\n");
        return 0;
    }

    while(true){
        c = idle();

		switch(c){
			case '8':
				for(int i=0; i<4; i++) {speed[i]=v;}
				//printf("speed set: %f\n",speed[0]);
				break;
			case '6':
				for(int i=0; i<4; i++) {speed[i]=1.5;}
				speed[1]=-speed[1];
				speed[3]=-speed[3];
				//printf("speed set: %f\n",speed[0]);
				break;
			case '4':
				for(int i=0; i<4; i++) {speed[i]=1.5;}
				speed[0]=-speed[0];
				speed[2]=-speed[2];
				//printf("speed set: %f\n",speed[0]);
				break;
			case '2':
				for(int i=0; i<4; i++) {speed[i]=-v;}
				//printf("speed set: %f\n",speed[0]);
				break;
			case '9':
				speed[0] = 2.0*v;
				speed[1] = 0.5*v;
				speed[2] = 2.0*v;
				speed[3] = 0.5*v;
				break;
			case '7':
				speed[0] = 0.5*v;
				speed[1] = 2.0*v;
				speed[2] = 0.5*v;
				speed[3] = 2.0*v;
				break;
			default:
				for(int i=0; i<4; i++) {speed[i]=0.0;}
				printf("speed set: %f\n",speed[0]);
				break;
		}

		//for(int i=0; i<4; i++) {speed[i]=v;}
        speed2pulseWidth(speed, pulseWidth, direction);
        outputPwmDir(pulseWidth, poChannel, direction, instantDoCtrl);
        printf("(Enter to stop.)\n");

        c = getchar();
        if ( c=='\n' || (c>'a' && c<'z') || (c>'A' && c<'Z') || (c>'0' && c<'9')){
            emergencyStop();
        }
    }

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
//                speed2pulseWidth
//====================================================
int speed2pulseWidth(double speed[4], PulseWidth *pulseWidth, uint8 direction[4])
{
	int i;
	double lo, rpm[4];

	for(i=0; i<4; i++){
        rpm[i] = speed[i]/PI*30.0*10.203; // rotor rpm
		if (rpm[i] == 0.0){
			(pulseWidth + i)->LoPeriod = 0.0;
            (pulseWidth + i)->HiPeriod = DUTY_CYCLE;
			continue;
		}
		if (rpm[i] > 0.0){
			direction[i] = (uint8)!(forwordDirection[i]);
		}
		else{
			rpm[i] = -rpm[i];
			direction[i] = (uint8)(forwordDirection[i]);
		}
		lo = (rpm[i]/32.0 + 5.0)/100.0*DUTY_CYCLE; // 0~24(V) => 0.05~0.98(Duty)
		if (lo > DUTY_CYCLE) {lo = DUTY_CYCLE;}
		(pulseWidth + i)->LoPeriod = lo;
		(pulseWidth + i)->HiPeriod = DUTY_CYCLE - lo;
	}
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


