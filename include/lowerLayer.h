#ifndef _LOWERLAYER_H_
#define _LOWERLAYER_H_

#include "parameters.h"
#include "signalProcessing.h"

// nominal tire model
#define Cx                740.0
#define mu_p              1.042
#define delta_lambda_bar  0.1      //5.0
#define delta_alpha_bar   50.0    //500.0
#define GAMMA             0.0

// Estimator parameters
#define a       180.0        //60.0(theoretically incorrect)( a > ln(2)/TS = 69.32 )
#define k1      10.0         //10.0
#define gamma1  2000.0       //2000.0
#define gamma2  100.0        //100.0
#define gamma3  50.0         //50.0
#define betaE   20.0         //20.0
#define epsilon 0.005

// Controller parameters
#define lambda_star 1.0
#define betat 10.0           //10.0
#define kapa1 5.0


// Emu refresh parameters
#define t_window_max 0.1
#define beta         10.0

// ================================= types declaration ================================
typedef struct Estimator_Data{  //for a single tire
	double mu_hat_dot = 0.0;
	double phi_dot = 0.0;
	double omega_hat_dot = 0.0;
	double omega_f_hat_dot = 0.0;

	double mu_hat = 0.0;
	double phi = 0.0;
	double omega_hat = 0.0;
	double omega_f_hat = 0.0;

	double pf0_plambda;   // partial(f0)/partial(lambda)
	double BT_hat, Br, C0, Cr, A0, W_bar, eta, k2;
	double eI_raw[2] = {0.0}, eI_LPF[2] = {0.0};
	double eI, ef, phi_tilde;
	double Emu = 0.0, Ei = 0.0, ti = 0.0;

	// constant need initialization
	double d; // d = 1-2ad
	double Fz0, e_factor;
	double Iw;
} ESTIMATOR_DATA;

typedef struct Controller_Data{  //for a single tire
	double Fad_old=0.0, Fad_dot=0.0;
	double ea;

	double T;
} CONTROLLER_DATA;

// =============================== functions declaration ==============================
int estimatorInit(ESTIMATOR_DATA *estimatorDataPtr, int index);
int estimator(ESTIMATOR_DATA *est, CG *c, TIRES *tires, double torq, double t, int index);
int torqueControl(CONTROLLER_DATA *ctrl, ESTIMATOR_DATA *est, CG *c, TIRES *tires, double Fad, int index);


#endif
