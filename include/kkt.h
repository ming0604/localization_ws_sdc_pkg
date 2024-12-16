#ifndef _KKT_H_
#define _KKT_H_

#include "parameters.h"

// =============================== types declaration ==============================
typedef struct Kkt_Data{
	//should be initialized when main startup
	double t_2 = T_2; // tf/2 = tr/2 = 0.275
	double u = 8;  //0.4
	double C[4] = {1000.0, 1000.0, 1000.0, 1000.0};
	double Fz0[4];
	double uFz0Square[4];
	//double QAA0[6][6];
	double QAA0inv[6][6];

	// output
	double Fad[4];
	double v[2];   // lagrange multipliers

} KKT_DATA;

// =============================== functions declaration ==============================
int kktInit(KKT_DATA *kktDataPtr);
int kktOptimization(double FX, double M, KKT_DATA *kktDataPtr);

#endif