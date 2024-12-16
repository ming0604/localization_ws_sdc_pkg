#ifndef _SIGNAL_H_
#define _SIGNAL_H_

#include "parameters.h"

// =============================== types declaration ==============================
typedef struct tracker_Data{
	bool    valid = false;
	double time = 0;
	double  poseRaw[3][4] = {0}, pose[3][4], originInv[3][4];
	double  linVelRaw[3], linVel[3];
	double  angVelRaw[3], angVel[3];
} TRACKER;

typedef struct amcl_Data{
	bool valid = false;
	double time = 0;
	double pose[3] = {0};
	double ekf_v[3] = {0};
	double x[5] = {0};
	double y[5] = {0};
	double yaw[5] = {0};
	double linVel_x[5] = {0};
	double linVel_y[5] = {0};
	double angVel_yaw[5] = {0};
	double ekf_vx[5] = {0};
	double ekf_vy[5] = {0};
	double ekf_vxl[3] = {0};
	double ekf_vyl[3] = {0};
	double VXLPF[3] = {0};
	double VYLPF[3] = {0};
	double rLPF[4] = {0};
	double imu = 0;
} AMCLPOSE;

typedef struct odom_Data{
	bool valid = false;
	double time = 0;
	double pose[3] = {0};
	double linVel[3] = {0};
	double angVel[3] = {0};
} ODOMPOSE;

typedef struct CG_Data{
	double px, py;   // position of tracker on CG_frame discripted in x,y
	double PX=TRACKER_X, PY=TRACKER_Y;   // position of tracker on CG_frame discripted in X,Y (these are constant.)
	double P_BX=BASELINK_X, P_BY=BASELINK_Y;
	double x, y;
	double X, Y;
	double dx[5] = {0}, dy[5] = {0};         // velocity of CG discripted in x,y .  Index 0 represent current value.
	double VX[3] = {0}, VY[3] = {0};         // velocity of CG discripted in X,Y
	double VXLPF[3] = {0}, VYLPF[3] = {0};   // velocity of CG discripted in X,Y (low pass filtered)
	double r[5] = {0};        // yaw rate
	double rLPF[4] = {0};     // yaw rate (low pass filtered)
	double psi;               // yaw angal
	double dr[5] = {0};       // yaw rate dot
	
	double ddx[5] = {0}, ddy[5] = {0};       // ddx = ax, ddy = ay
	double AX, AY;
	double VX_dot, VY_dot;                   // used by estimator

	double dx_r[3] = {0};
	double dx_r_LPF[3] = {0};
	double x_r[2] = {0};
} CG;

typedef struct Single_Tire_Data{
	double VWX, VWY; // longitudinal velocity, lateral velocity calculated from cg data
	double omega[3];    // angular velocity
	double omegaLPF[3]; // angular velocity (low pass filtered)
	double lambda;  // slip ratio
	double alpha;  // slip angel

	/*encoder counter*/
	int    cntOld = 0;
	int    cntNew = 0;
} SINGLE_TIRE;

typedef struct Tires_Data{
	//double tf_2 = 0.275;
	//double tr_2 = 0.275;
	//double lf = 0.302;
	//double lr = 0.310;

	double s[4] = {T_2, -T_2, T_2, -T_2};
	double l[4] = {-LR, -LR, LF, LF};
	double R = TIRE_RADIUS; // tire radius
	SINGLE_TIRE theTire[4];
} TIRES;


// =============================== functions declaration ==============================
int readT0inv(const char *filename, char *str);
int str2originInv(char *str, double originInv[3][4]);
void dataShift(double *data, int length);
void differentiator(double *x, double *y);
void differentiator_amcl(double *x, double *y);
void LPfilter(double *x, double *y);
void LPfilter_2(double *x, double *y);
void LPfilter_1_order(double *x, double *y);
void LPfilter_2_order(double *x, double *y);
//int vectorTransform(double vec[3], double originInv[3][4], double vecRaw[3]);
//int matrixTransform(double pose[3][4], double originInv[3][4], double poseRaw[3][4]);
//int str2rawdata(char *str, TRACKER *tckPtr);
int shm2tck(void *ptr, TRACKER *tckPtr);
// int ros2amcl(void *ptr, AMCLPOSE *amclPtr);
int ros2amcl(double *ptr, AMCLPOSE *amclPtr);
int tck2cg(TRACKER tck, CG *cgPtr);
// int tck2cg(TRACKER tck, CG *cgPtr, AMCLPOSE *amclPtr);
int amcl2cg(AMCLPOSE *amclPtr, CG *cgaPtr);
// int amcl2cg(AMCLPOSE *amclPtr, CG *cgaPtr, CG *cgPtr);
int cg2tires(CG cg, TIRES *tires);
int signal2cg(TRACKER tck, AMCLPOSE *amclPtr, CG *cgPtr, CG *cgaPtr, double t);
double sigmoid(double x, double k);
void smoothTransition(CG *cgPtr, CG *cgaPtr, double t);

#endif
