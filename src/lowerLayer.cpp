#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cfloat>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "lowerLayer.h"

inline double mySq(double x) {return x*x;}
inline double mySgn(double x) {return (x<0.0) ? -1.0 : 1.0;}
inline bool IsNumber(double x) {
	// This looks like it should always be true, 
	// but it's false if x is a NaN.
	return (x == x); 
}
inline bool IsFiniteNumber(double x) {
	return (x <= DBL_MAX && x >= -DBL_MAX); 
}

// LPF     0.09516*z^-1
// --- = ---------------
// raw   1 - 0.9048*z^-1
inline void eI_filter(double *raw, double *LPF) {
	LPF[0] = 0.09516*raw[1] + 0.9048*LPF[1];
}

// ==========================================================
//                        estimatorInit
// ==========================================================
int estimatorInit(ESTIMATOR_DATA *est, int index) // call this before timer starts
{
	memset(est, 0, sizeof(ESTIMATOR_DATA));

	est->d = 1.0-2*exp(-a*TS);
	switch(index){
		case 0: // tire 1
			est->Fz0 = MASS*GRAVITY*LR/(LF+LR)*0.5;
			est->Iw = 0.1277;
			est->e_factor = -(0.1277/TIRE_RADIUS/(est->Fz0));
			break;
		case 1: // tire 2
			est->Fz0 = MASS*GRAVITY*LR/(LF+LR)*0.5;
			est->Iw = 0.1157;
			est->e_factor = -(0.1157/TIRE_RADIUS/(est->Fz0));
			break;
		case 2: // tire 3
			est->Fz0 = MASS*GRAVITY*LF/(LF+LR)*0.5;
			est->Iw = 0.1383;
			est->e_factor = -(0.1383/TIRE_RADIUS/(est->Fz0));
			break;
		case 3: // tire 4
			est->Fz0 = MASS*GRAVITY*LF/(LF+LR)*0.5;
			est->Iw = 0.1332;
			est->e_factor = -(0.1332/TIRE_RADIUS/(est->Fz0));
			break;
	}

	return 1;
}


// ==========================================================
//                estimator sub functions
// ==========================================================
int refreshErrorSignal(ESTIMATOR_DATA *est, double omega, int index)
{
	est->eI_raw[1] = est->eI_raw[0];
	est->eI_raw[0] = (omega - est->omega_hat)*(est->e_factor);
	est->eI_LPF[1] = est->eI_LPF[0];
	eI_filter(est->eI_raw, est->eI_LPF);
	est->eI = est->eI_LPF[0];

	est->ef = (omega - est->omega_f_hat)*(est->e_factor);
	est->phi_tilde = est->phi - est->eI;
	return 1;
}

int refreshEmu(ESTIMATOR_DATA *est, double t)
{
	double t_temp;
	double bound_temp;
	double bound_trivial;


	// Ei kept
	if((t_temp = t - est->ti) > t_window_max){   // out of window => decay
		est->Emu = est->Ei*exp(-beta*(t_temp-t_window_max));
		//return 0;
	}
	else{                                                     // inside window => hold
		est->Emu = est->Ei;
		//return 0;
	}

	// Ei reset
	if( (est->Emu) < (bound_temp = a/(est->d)*fabs(est->eI)) ){
		est->Emu = bound_temp;
		est->Ei = bound_temp;
		est->ti = t;
		//return 1; //may be used to record reset times
	}
	else{
		//return 0;
	}

	bound_trivial = 1.0 + fabs(est->mu_hat);
	if ((est->Emu) > bound_trivial ){
		(est->Emu) = bound_trivial;
		//est->Ei = bound_trivial;
		//est->ti = t;
	}
	return 0;
}

int updateEstimator(ESTIMATOR_DATA *est, CG *c, TIRES *tires, double torq, int index) // refresh BT_hat, Br, C0, Cr // refresh A0, W_bar
{
	double oneMinusLambdaAbs = 1.0-fabs(tires->theTire[index].lambda);
	double frontFac = oneMinusLambdaAbs/(est->Fz0);
	double firstPart = (torq - (est->mu_hat)*(est->Fz0)*TIRE_RADIUS) / (tires->theTire[index].omegaLPF[0]) / (est->Iw);
	double secondPart = (c->VX_dot)/tires->theTire[index].VWX;
	double VW2 = mySq(tires->theTire[index].VWX) + mySq(tires->theTire[index].VWY);

	double s_prime = mu_p*(est->Fz0)*oneMinusLambdaAbs*0.5/Cx/fabs(tires->theTire[index].lambda);

	est->BT_hat = frontFac*(firstPart - secondPart);
	est->Br = -frontFac*(tires->s[index])/(tires->theTire[index].VWX);

	est->C0 = -((tires->theTire[index].VWX)*(c->VY_dot) - (tires->theTire[index].VWY)*(c->VX_dot)) / (est->Fz0) / VW2;
	est->Cr = -((tires->theTire[index].VWX)*(tires->l[index]) - (tires->theTire[index].VWY)*(tires->s[index])) / VW2;

	if (s_prime > 1.0){
		est->pf0_plambda = Cx/mySq(oneMinusLambdaAbs);
	}
	else{
		double s_prime_partial = - mu_p*(est->Fz0)*0.5/Cx/mySq(tires->theTire[index].lambda)*mySgn(tires->theTire[index].lambda);
		est->pf0_plambda = Cx/mySq(oneMinusLambdaAbs)*(2.0*s_prime - mySq(s_prime)) + Cx*(tires->theTire[index].lambda)/oneMinusLambdaAbs*s_prime_partial*2.0*(1.0-s_prime);
	}
	

	est->A0 = -(oneMinusLambdaAbs / (tires->theTire[index].omegaLPF[0]))*TIRE_RADIUS/est->Iw;
	est->W_bar = delta_lambda_bar*fabs(est->pf0_plambda)*fabs(est->BT_hat + (est->Br)*(c->dr[0])) + delta_alpha_bar*(est->C0 + (est->Cr)*(c->dr[0])) + GAMMA;

	return 1;
}

int updateUpdateLaw(ESTIMATOR_DATA *est)
{
	double K = 0.0;//10.0*mySq(est->phi_tilde);
	double alpha1 = (betaE*gamma1*0.5) + gamma1*(1.0+delta_lambda_bar)*fabs((est->pf0_plambda)*(est->A0));

	est->eta = ( gamma2*(est->ef) - gamma3*(est->phi_tilde) + est->eI_raw[0] + 0.5*K )/gamma1;

	// case I
	if( fabs(est->phi_tilde) >= epsilon ){
		est->k2 = 0.0;
		est->phi_dot = -0.5*betaE*(est->phi_tilde) - a*(est->eI_raw[0]) - (alpha1*mySq(est->Emu) + (gamma1*(est->W_bar) + K)*(est->Emu))/gamma3/(est->phi_tilde);
		return 1;
	}
	// case II
	else if( fabs(est->ef) > epsilon/k1 ){
		est->k2 = 2*gamma3*k1/gamma2;
		est->phi_dot = -(alpha1*mySq(est->Emu) + (gamma1*(est->W_bar)+K)*(est->Emu) + gamma3*a*(est->phi_tilde)*(est->eI) + 0.5*betaE*gamma3*mySq(est->phi_tilde))
		                                        / (gamma3*(est->phi_tilde) - gamma2*(est->k2)*(est->ef));
		return 2;
	}
	// case III
	else{
		est->k2 = 0.0;
		est->phi_dot = -0.5*betaE*(est->phi_tilde) - a*(est->eI_raw[0]) - (alpha1*mySq(est->Emu) + (gamma1*(est->W_bar) + K)*(est->Emu))/gamma3/epsilon*mySgn(est->phi_tilde);
		return 3;
	}
}


// ==========================================================
//                 estimator main function
// ==========================================================
int estimator(ESTIMATOR_DATA *est, CG *c, TIRES *tires, double torq, double t, int index) // index => tire number 0~3
{
	//static int Emu_reset_times[4] = {0};
	static int phiCaseNew[4], phiCaseOld[4] = {0};
	double temp;

	

	/* refresh error signal */
	refreshErrorSignal(est, tires->theTire[index].omegaLPF[0], index);
	/* update BT_hat, Br, C0, Cr, partial(f0)/partial(lambda), A0, W_bar */
	updateEstimator(est, c, tires, torq, index);
	/* refresh Emu */
	refreshEmu(est, t);
	/* calculate eta, choose k2 and phi */
	phiCaseNew[index] = updateUpdateLaw(est);
	if (phiCaseNew[index] != phiCaseOld[index]){
		phiCaseOld[index] = phiCaseNew[index];
		//printf("tire %d phi case changed to %d\n", index, phiCaseNew[index]);
	}

	/* calculate  mu_hat_dot, phi_dot, omega_hat_dot, omega_f_hat_dot */
	est->mu_hat_dot = est->pf0_plambda*(est->BT_hat + est->Br*(c->dr[0])) + est->eta;
	// est->phi_dot has already been chosen in updateUpdateLaw()
	temp = (torq - (est->mu_hat)*(est->Fz0)*TIRE_RADIUS)/(est->Iw);
	est->omega_hat_dot = temp + a*(tires->theTire[index].omegaLPF[0] - est->omega_hat);
	est->omega_f_hat_dot = temp + k1*(tires->theTire[index].omegaLPF[0] - est->omega_f_hat) + est->k2/(est->e_factor)*(est->phi_dot);

	/* integration */
	est->mu_hat      += est->mu_hat_dot * TS;
	est->phi         += est->phi_dot * TS;
	est->omega_hat   += est->omega_hat_dot * TS;
	est->omega_f_hat += est->omega_f_hat_dot * TS;
	

	return 1;
}


// ==========================================================
//                    torque controller
// ==========================================================
int torqueControl(CONTROLLER_DATA *ctrl, ESTIMATOR_DATA *est, CG *c, TIRES *tires, double Fad, int index)
{
	double zeta;
	double kapa;

	ctrl->ea = (est->mu_hat)*(est->Fz0) - Fad;
	ctrl->Fad_dot = (Fad - ctrl->Fad_old)/TS;
	ctrl->Fad_old = Fad;

	if(/*fabs(tires->theTire[index].lambda) > lambda_star*/false){
		// unused terms had already been eliminated
		zeta = fabs(est->A0)*(fabs(ctrl->ea) + (est->Emu)*(est->Fz0))*mySgn(tires->theTire[index].lambda) + kapa1*(tires->theTire[index].lambda);
		ctrl->T = Fad*TIRE_RADIUS + tires->theTire[index].omegaLPF[0]*(est->Iw)*( c->VX_dot/tires->theTire[index].VWX - 
		          (est->Fz0)/(1.0-fabs(tires->theTire[index].lambda))*((est->Br)*(c->dr[0]) + zeta/(est->Fz0)) );
		
		return 0;
	}
	else{
		kapa = est->pf0_plambda*(est->A0)+0.5*betat;
		zeta = kapa*(ctrl->ea) + (est->eta)*(est->Fz0);
		ctrl->T = Fad*TIRE_RADIUS + tires->theTire[index].omegaLPF[0]*(est->Iw)*( c->VX_dot/tires->theTire[index].VWX - 
		          (est->Fz0)/(1.0-fabs(tires->theTire[index].lambda))*( (est->Br)*(c->dr[0]) + (zeta - ctrl->Fad_dot)/(est->Fz0)/(est->pf0_plambda) ) );

		return 1;
	}
}



