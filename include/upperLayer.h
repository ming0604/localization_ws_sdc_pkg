#ifndef _UPPERLAYER_H_
#define _UPPERLAYER_H_

#include "signalProcessing.h"

// controller parameters
// P-0.2 8 10 8 15
// P-0.1 8 10 8 10
// P-0.4
#define ks1_tck 3.0  //0.4 3.0 2.0
#define ks1_amcl 1.2   //2.0 0.3
#define ks1_s 0.6

#define ks2 3.6  //3.6

// #define epsilonSV1_U 0.2  //0.2
// #define epsilonSV1_L 0.3  //0.3
// #define epsilonSV2_U 0.3 //0.3
// #define epsilonSV2_L 0.4  //0.4
#define epsilonSV1_U 0.4  //0.4
#define epsilonSV1_L 0.5  //0.5
#define epsilonSV2_U 0.4 //0.3
#define epsilonSV2_L 0.5  //0.4
#define C1 0.0 //0.03 
#define C2 0.05 //0.05
#define lemda1_tck 0.4 //0.4
#define lemda1_amcl 0.45
#define lemda1_s 4.5 //2.5
#define lemda1_f 4.5 //3.0
// #define lemda2 27.0 //15 /

// #define lemda1_s 0.35
// #define lemda1_f 0.35
// #define lemda2 23.0 //15 20 23 25 27
#define lemda2_s 15.0 //18.0
#define lemda2_f 27.0 //27.0 22.0
//v12 29 
//v14 27 
//v16 25 
//v18 20



#define d2_bar 0.001//0.01
#define epsilonPlus 0.1

//lemda 2.2 

//#define COMPENSATE_RATE 0//0.0~1.0

// gain auto adjusting
#define Vadj   0.1
#define A_low  0.5
#define ksx_low 4.0
#define kx_low 5.0


// =============================== types declaration ==============================
typedef struct Desired_Trajectory_Data{
	double xd,yd;
	double dxd,dyd;
	double ddxd, ddyd;
	double dddxd, dddyd;
	double psid, rd, drd;
	double curvature;
} DESIRED_TRAJECTORY;




typedef struct UpperLayerData{
	DESIRED_TRAJECTORY   desiredTrajectory;


	double               yd_r,dyd_r,ddxd_r;
	double 				 xd_r[2] = {0};
	double 				 dxd_r[2] = {0};

    double               y_r[3]={0};
    double 				 y_r_LPF[3]={0};
    
    double               dy_r[3]={0};
    double				 dy_r_bridge[3]={0};
    double 				 dy_r_LPF[3]={0};

    double               ddy_r[3]={0};
    double               ddy_r_bridge[3]={0};
    double 				 ddy_r_LPF[3]={0};

    double               psi_r[3]={0};
    double 				 psi_r_LPF[3]={0};
    double               r_r[3] = {0};
    double 				 r_r_bridge[2] = {0};
    double 				 delta_x1,delta_x2;
 	double				 integral_delta_x1[2] = {0};
 	double				 integral_delta_x2[2] = {0};
    double               c_x,c_y; //road error reference point
    int                  ccount;// calculate road error times

    double               SV1,SV2;
	double				 lemda1, lemda2;
	double 				 ks1;
	double               FX, M;  // M = Mlong = Moment induced by longitudinal force

	double 				 F_D1, F_ks1sat, F_lemda1SV1, F_dy_r_dx_r_curvature;
	double   			 M_1, M_2, M_3, M_D2, M_ks2sat, M_lemda2SV2;

	double 				 law[5]={0};
	double 				 law_dot[5]={0};
	double				 eta_last_matric;
} UPPERLAYER_DATA;



// =============================== functions declaration ==============================
// generate upperLayerDataPtr->desiredTrajectory from t
int genDesiredTrajectory1(double t, UPPERLAYER_DATA *upperLayerDataPtr); // atan with fixed parameter
int genDesiredTrajectory2(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double W, double b, double v); // atan adjustable (constant speed)
int genDesiredTrajectory3(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double v, double t1); // straight line
int genDesiredTrajectory4(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double W, double b, double v, double t1); // tanh adjustable (trapezoidal speed)
int genDesiredTrajectory5(double t, UPPERLAYER_DATA *upperLayerDataPtr, double v, const bool B);
int genDesiredTrajectory6(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double W, double b, double v);
int genDesiredTrajectory7(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L,double v);
int genDesiredTrajectory8(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L,double v,double R);

// generate upperLayerDataPtr->previewTrajectory from upperLayerDataPtr->desiredTrajectory
//int genPreviewTrajectory(UPPERLAYER_DATA *upperLayerDataPtr, CG *c);

void derivative(double *x, double *y);
double sqrt3(double a);
double sgn(double in);
int calculate_road_error(CG cg, UPPERLAYER_DATA *u,double sec,double v);
int calculate_road_error_genDesiredTrajectory7(CG cg, UPPERLAYER_DATA *u,double sec,double L,double v);
int calculate_road_error_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double sec,double L,double v,double R);

//int upperLayerControl(CG cg, UPPERLAYER_DATA *upperLayerDataPtr,double k_gain[][8], double dk_gain[][8],double sec,double L, double v);
int upperLayerControl(CG *cgPtr, UPPERLAYER_DATA *u,double k_gain[][8],double dk_gain[][8],double Pe[][6],double b[],double sec,double L, double v);
int upperLayerControl_genDesiredTrajectory8(CG *cgPtr, UPPERLAYER_DATA *u,double k_gain[][8],double dk_gain[][8],double k_gain_plus[][8],double dk_gain_plus[][8],double Pe[][6],double b[],double sec,double L, double v,double R);

#endif
