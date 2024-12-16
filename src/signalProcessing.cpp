#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "signalProcessing.h"

//========================================================================
//                          tracker origin init
//========================================================================
int readT0inv(const char *filename, char *str)
{
	FILE *fd;
	long size;

	fd = fopen(filename, "r");
	if(fd == NULL){
		printf("fopen() failed.\n");
		return 0;
	}
	fseek(fd, 0, SEEK_END);
	size = ftell(fd);
	if(size == -1){
		printf("content of %s is invalid.\n", filename);
		return 0;
	}
	//printf("size: %ld\n", size);
	fseek(fd, 0, SEEK_SET);
	if(fread(str, sizeof(char), size, fd) != size){
		printf("error: fread()");
		return 0;
	}
	str[size] = '\0';

	return 1;
}

int str2originInv(char *str, double originInv[3][4])
{
	int i=0;
	// T0inv
	for(int m=0; m<3; m++){
		for(int n=0; n<4; n++){
			originInv[m][n] = atof(str+i);
			while(str[i] != ' '){
				i++;                   // pass digits
			}
			i++;                       // pass ' '
		}
	}
	// Rxinv*T0inv
	for(int m=1; m<3; m++){
		for(int n=0; n<4; n++){
			originInv[m][n] = -originInv[m][n];
		}
	}
	return 1;
}



//========================================================================
//                         tracker data operations
//========================================================================
int vectorTransform(double vec[3], double originInv[3][4], double vecRaw[3])
{
	// vec = originInv*vecRaw
	for(int m=0; m<3; m++){
		vec[m] = 0;
		for(int i=0; i<3; i++)
			vec[m] += originInv[m][i]*vecRaw[i];
	}
	return 1;
}

int matrixTransform(double pose[3][4], double originInv[3][4], double poseRaw[3][4])
{
	// pose = originInv*poseRaw
	double R1P2[3];
	double P2[3];

	P2[0] = poseRaw[0][3];
	P2[1] = poseRaw[1][3];
	P2[2] = poseRaw[2][3];
	for(int m=0; m<3; m++){
		for(int n=0; n<3; n++){
			pose[m][n] = 0;
			for(int i=0; i<3; i++)
				pose[m][n] += originInv[m][i]*poseRaw[i][n];
		}
	}
	vectorTransform(R1P2, originInv, P2);
	for(int m=0; m<3; m++)
		pose[m][3] = R1P2[m] + originInv[m][3];

	return 1;
}

/*
int str2rawdata(char *str, TRACKER *tckPtr)
{
	int i=0;

	if(str[0] == '1'){
		tckPtr->valid = true;
		i = 2;
		//_______________ pose _______________
		for(int m=0; m<3; m++){
			for(int n=0; n<4; n++){
				tckPtr->poseRaw[m][n] = atof(str+i);
				while(str[i] != ' '){
					i++;                   // pass digits
				}
				i++;                       // pass ' '
			}
		}
		//______________ linVel ______________
		for(int m=0; m<3; m++){
			tckPtr->linVelRaw[m] = atof(str+i);
			while(str[i] != ' '){
				i++;                   // pass digits
			}
			i++;                       // pass ' '
		}
		//______________ angVel ______________
		for(int m=0; m<3; m++){
			tckPtr->angVelRaw[m] = atof(str+i);
			while(str[i] != ' '){
				i++;                   // pass digits
			}
			i++;                       // pass ' '
		}
	}
	else{
		tckPtr->valid = false;
	}

	return 1;
}
*/

int shm2rawdata(void* ptr, TRACKER *tckPtr)
{
	if(*(bool*)ptr){
		tckPtr->valid = true;
		//_______________ pose _______________
		for(int m=0; m<3; m++){
			for(int n=0; n<4; n++){
				tckPtr->poseRaw[m][n] = ((double*)((bool*)ptr+1))[m*4+n];
			}
		}
		//______________ linVel ______________
		for(int m=0; m<3; m++){
			tckPtr->linVelRaw[m] = ((double*)((bool*)ptr+1))[m+12];
		}
		//______________ angVel ______________
		for(int m=0; m<3; m++){
			tckPtr->angVelRaw[m] = ((double*)((bool*)ptr+1))[m+15];
		}
	}
	else{
		tckPtr->valid = false;
	}

	return 1;
}

int shm2tck(void *ptr, TRACKER *tckPtr)
{
	shm2rawdata(ptr, tckPtr);
	matrixTransform(tckPtr->pose, tckPtr->originInv, tckPtr->poseRaw);         // pose = originInv*poseRaw
	vectorTransform(tckPtr->linVel, tckPtr->originInv, tckPtr->linVelRaw);     // linVel = originInv*linVelRaw
	vectorTransform(tckPtr->angVel, tckPtr->originInv, tckPtr->angVelRaw);

	return 1;
}


//========================================================================
//                            signal filter
//========================================================================
void dataShift(double *data, int length)
{
	for (int i=0; i<length-1; i++){
		data[length-1-i] = data[length-2-i];
	}
	data[0] = 0;
}

void differentiator(double *x, double *y)
{
	//low pass filter hz = 2.59 order 3 is same with Cheng   by Ryan
	// y[0] = 0.4304*x[0] +0.9265*x[1] -1.091*x[2] -0.2663*x[3]+2.178*y[1]-1.582*y[2] +0.3829*y[3];

	// low pass filter hz = 3 by Ryan
	y[0] = 0.6413*x[0] +1.307*x[1] -1.58*x[2] -0.3681*x[3]+2.072*y[1]-1.431*y[2] +0.3296*y[3];

	//low pass filter hz = 4 by Ryan
	// y[0] = 1.394*x[0] +2.472*x[1] -3.202*x[2] -0.6641*x[3]+1.831*y[1]-1.117*y[2] +0.2272*y[3];

	//low pass filter hz = 5 by Ryan
	// y[0] = 2.493*x[0] +3.824*x[1] -5.331*x[2] -0.9861*x[3]+1.617*y[1]-0.8716*y[2] +0.1566*y[3];

	//low pass filter hz = 6 by Ryan
	// y[0] = 3.932*x[0] +5.179*x[1] -7.818*x[2] -1.293*x[3]+1.43*y[1]-0.6815*y[2] +0.1083*y[3];	
}

void differentiator_amcl(double *x, double *y)
{
	// low pass filter a = 3.56, cutoff 1.798 hz
	// y[0] = 0.1579*x[0] +0.3766*x[1] -0.4216*x[2] -0.1129*x[3]+2.399*y[1]-1.918*y[2] +0.5112*y[3];

	// low pass filter a = 2.16, cutoff 1.09 hz
	// y[0] = 0.03764*x[0] +0.09841*x[1] -0.1053*x[2] -0.03071*x[3]+2.619*y[1]-2.287*y[2] +0.6655*y[3];
	
	// low pass filter: 3 order 
	// y[0] = 0.02671*x[0] +0.06992*x[1] -0.07481*x[2] -0.02182*x[3]+2.626*y[1]-2.295*y[2] +0.6672*y[3];
	// low pass filter: 3 order 
	// y[0] = 0.02685*x[0] +0.07027*x[1] -0.07519*x[2] -0.02193*x[3]+2.626*y[1]-2.294*y[2] +0.6667*y[3];

	// butterworth filter: 2 order, cutoff 1.5 hz
	// y[0] = 0.4247*x[0] -0.01846*x[1] -0.4063*x[2]+1.867*y[1]-0.8752*y[2];
	
	// butterworth filter: 2 order, cutoff 1.3 hz
	// y[0] = 0.3209*x[0] -0.01212*x[1] -0.3088*x[2]+1.885*y[1]-0.8909*y[2];
	
	// butterworth filter: 2 order, cutoff 1.0 hz
	// y[0] = 0.1916*x[0] -0.005593*x[1] -0.186*x[2]+1.911*y[1]-0.915*y[2];
	
	// low pass filter: 2 order, cutoff 0.871 hz
	y[0] = 0.3855*x[0] -0.02345*x[1] -0.362*x[2]+1.82*y[1]-0.8284*y[2];

	// low pass filter: 2 order, cutoff 0.7 hz
	// y[0] = 0.2241*x[0] -0.01*x[1] -0.2141*x[2]+1.868*y[1]-0.872*y[2];
	
	// low pass filter: 2 order, cutoff 0.6 hz
	// y[0] = 0.1677*x[0] -0.006475*x[1] -0.1612*x[2]+1.885*y[1]-0.8886*y[2];

	// low pass filter: 2 order, cutoff 0.5 hz
	// y[0] = 0.1157*x[0] -0.003708*x[1] -0.1119*x[2]+1.905*y[1]-0.9069*y[2];
	
	// low pass filter: 1 order 
	// y[0] = 6.619*x[0] -6.619*x[1]+0.9338*y[1];

	// y[0] = (x[0] - x[1])/0.01;
}

void LPfilter(double *x, double *y)
{
	//hz =  2(hz) order:1
	// y[0] = 0.8781*y[1]+0.06227*x[0] + 0.05963*x[1]; 

	//hz = 2.59
	// y[0] = 0.8496*y[1]+0.07725*x[0] + 0.07316*x[1];

	//hz = 3
	y[0] = 0.8278*y[1]+0.08882*x[0] + 0.0834*x[1]; 

	//hz = 4
	// y[0] = 0.7772*y[1]+0.1161*x[0] + 0.1067*x[1]; 

	//hz = 5
	// y[0] = 0.7298*y[1]+0.1422*x[0] + 0.128*x[1]; 

	//hz = 6
	// y[0] = 0.6852*y[1]+0.1673*x[0] + 0.1475*x[1]; 

	//best cutoff freq. 6.42(Hz) Cheng
	//y[0] = 0.0485*x[0] + 0.1432*x[1] + 0.0259*x[2] + 1.0670*y[1] - 0.2846*y[2]; 

}
void LPfilter_2(double *x, double *y){
	//hz =  2(hz) order:1
	y[0] = 0.8781*y[1]+0.06227*x[0] + 0.05963*x[1]; 
}

// cutoff freq. 10(Hz)
//  y     (0.0485 + 0.1432z^-1 + 0.0259z^-2)
// --- = ------------------------------------
//  x        (1 - 1.0670z^-1 + 0.2846z^-2)

// cutoff freq. 15(Hz)
//  y     (0.09448 + 0.2412 z^-1 + 0.03682 z^-2)
// --- = ---------------------------------------
//  x        (1 - 0.7793 z^-1 + 0.1518 z^2)

void LPfilter_1_order(double *x, double *y)
{
	y[0] = x[0];
	//y[0] = 0.8781*y[1]+0.06227*x[0] + 0.05963*x[1]; //cutoff freq. 2(hz) order:1
}

//========================================================================
//                                tck2cg
//========================================================================
int tck2cg(TRACKER tck, CG *cgPtr)
{
	// shift data history
	dataShift(cgPtr->dx, 5);
	dataShift(cgPtr->dy, 5);
	dataShift(cgPtr->r, 5);
	dataShift(cgPtr->ddx, 5);
	dataShift(cgPtr->ddy, 5);
	dataShift(cgPtr->dr, 5);


	dataShift(cgPtr->VX, 3);
	dataShift(cgPtr->VY, 3);
	dataShift(cgPtr->VXLPF, 3);
	dataShift(cgPtr->VYLPF, 3);
	dataShift(cgPtr->rLPF, 4);
	


	// get CG data
	cgPtr->psi = atan2(tck.pose[1][0], tck.pose[0][0]); // atan return -PI ~ PI
	//if (cgPtr->psi<-PI/2) { cgPtr->psi += 2*PI; }

	cgPtr->px = cgPtr->PX*cos(cgPtr->psi)-cgPtr->PY*sin(cgPtr->psi);
	cgPtr->py = cgPtr->PX*sin(cgPtr->psi)+cgPtr->PY*cos(cgPtr->psi);

	cgPtr->x = cgPtr->PX + tck.pose[0][3] - cgPtr->px;
	cgPtr->y = cgPtr->PY + tck.pose[1][3] - cgPtr->py;
	cgPtr->X = cgPtr->x*cos(cgPtr->psi) + cgPtr->y*sin(cgPtr->psi);
	cgPtr->Y = -cgPtr->x*sin(cgPtr->psi) + cgPtr->y*cos(cgPtr->psi);
	cgPtr->r[0] = tck.angVel[2];
	cgPtr->dx[0] = tck.linVel[0] + cgPtr->py*tck.angVel[2];
	cgPtr->dy[0] = tck.linVel[1] - cgPtr->px*tck.angVel[2];
	cgPtr->VX[0] = cgPtr->dx[0]*cos(cgPtr->psi) + cgPtr->dy[0]*sin(cgPtr->psi);
	cgPtr->VY[0] = -cgPtr->dx[0]*sin(cgPtr->psi) + cgPtr->dy[0]*cos(cgPtr->psi);

	// differentiate
	differentiator(cgPtr->dx, cgPtr->ddx);
	differentiator(cgPtr->dy, cgPtr->ddy);
	differentiator(cgPtr->r, cgPtr->dr);

	// low pass filter
	LPfilter(cgPtr->VX, cgPtr->VXLPF);
	LPfilter(cgPtr->VY, cgPtr->VYLPF);
	LPfilter(cgPtr->r, cgPtr->rLPF);



	cgPtr->AX = cgPtr->ddx[0]*cos(cgPtr->psi) + cgPtr->ddy[0]*sin(cgPtr->psi);
	cgPtr->AY = -cgPtr->ddx[0]*sin(cgPtr->psi) + cgPtr->ddy[0]*cos(cgPtr->psi);

	cgPtr->VX_dot = cgPtr->AX + (cgPtr->rLPF[0])*(cgPtr->VYLPF[0]);
	cgPtr->VY_dot = cgPtr->AY - (cgPtr->rLPF[0])*(cgPtr->VXLPF[0]);
	
	//cgPtr->dx_r = cgPtr->VXLPF[0]+(cgPtr->rLPF[0])*(cgPtr->Y);  //X_dot

	return 1;
}

//========================================================================
//                                tck2cg
//========================================================================
int amcl2cg(AMCLPOSE *amclPtr, CG *cgaPtr)
{
	// shift data history
	dataShift(cgaPtr->dx, 5);
	dataShift(cgaPtr->dy, 5);
	dataShift(cgaPtr->r, 5);
	dataShift(cgaPtr->ddx, 5);
	dataShift(cgaPtr->ddy, 5);
	dataShift(cgaPtr->dr, 5);


	dataShift(cgaPtr->VX, 3);
	dataShift(cgaPtr->VY, 3);
	dataShift(cgaPtr->VXLPF, 3);
	dataShift(cgaPtr->VYLPF, 3);
	dataShift(cgaPtr->rLPF, 4);

	dataShift(amclPtr->x, 5);
	dataShift(amclPtr->y, 5);
	dataShift(amclPtr->yaw, 5);
	dataShift(amclPtr->linVel_x, 5);
	dataShift(amclPtr->linVel_y, 5);
	dataShift(amclPtr->angVel_yaw, 5);
	dataShift(amclPtr->ekf_vx, 5);
	dataShift(amclPtr->ekf_vy, 5);
	dataShift(amclPtr->ekf_vxl, 3);
	dataShift(amclPtr->ekf_vyl, 3);
	dataShift(amclPtr->VXLPF, 3);
	dataShift(amclPtr->VYLPF, 3);
	dataShift(amclPtr->rLPF, 4);
	
	amclPtr->x[0] = amclPtr->pose[0];
	amclPtr->y[0] = amclPtr->pose[1];
	amclPtr->yaw[0] = amclPtr->pose[2];

	amclPtr->ekf_vx[0] = amclPtr->ekf_v[0];
	amclPtr->ekf_vy[0] = amclPtr->ekf_v[1];

	LPfilter(amclPtr->ekf_vx, amclPtr->ekf_vxl);
	LPfilter(amclPtr->ekf_vy, amclPtr->ekf_vyl);

	differentiator_amcl(amclPtr->x, amclPtr->linVel_x);
	differentiator_amcl(amclPtr->y, amclPtr->linVel_y);
	differentiator_amcl(amclPtr->yaw, amclPtr->angVel_yaw);
	
	// get CG data
	cgaPtr->psi = amclPtr->pose[2]; // atan return -PI ~ PI
	// cgaPtr->psi = atan2(amclPtr->pose[1], amclPtr->pose[0]); // atan return -PI ~ PI

	cgaPtr->px = cgaPtr->P_BX*cos(cgaPtr->psi)-cgaPtr->P_BY*sin(cgaPtr->psi);
	cgaPtr->py = cgaPtr->P_BX*sin(cgaPtr->psi)+cgaPtr->P_BY*cos(cgaPtr->psi);

	cgaPtr->x = cgaPtr->P_BX + amclPtr->pose[0] - cgaPtr->px;
	cgaPtr->y = cgaPtr->P_BY + amclPtr->pose[1] - cgaPtr->py;

	// smoothTransition(cgPtr, cgaPtr, t);
	
	cgaPtr->X = cgaPtr->x*cos(cgaPtr->psi) + cgaPtr->y*sin(cgaPtr->psi);
	cgaPtr->Y = -cgaPtr->x*sin(cgaPtr->psi) + cgaPtr->y*cos(cgaPtr->psi);
	cgaPtr->r[0] = amclPtr->imu;
	cgaPtr->dx[0] = amclPtr->linVel_x[0] + cgaPtr->py*amclPtr->imu;
	cgaPtr->dy[0] = amclPtr->linVel_y[0] - cgaPtr->px*amclPtr->imu;
	// cgaPtr->r[0] = amclPtr->angVel_yaw[0];
	// cgaPtr->dx[0] = amclPtr->linVel_x[0] + cgaPtr->py*amclPtr->angVel_yaw[0];
	// cgaPtr->dy[0] = amclPtr->linVel_y[0] - cgaPtr->px*amclPtr->angVel_yaw[0];
	cgaPtr->VX[0] = cgaPtr->dx[0]*cos(cgaPtr->psi) + cgaPtr->dy[0]*sin(cgaPtr->psi);
	cgaPtr->VY[0] = -cgaPtr->dx[0]*sin(cgaPtr->psi) + cgaPtr->dy[0]*cos(cgaPtr->psi);

	// differentiate
	differentiator(cgaPtr->dx, cgaPtr->ddx);
	differentiator(cgaPtr->dy, cgaPtr->ddy);
	differentiator(cgaPtr->r, cgaPtr->dr);

	// printf("%lf, %lf, %lf\n", amclPtr->linVel_x[0], amclPtr->linVel_y[0], amclPtr->yaw[0]);
	// printf("%lf, %lf, %lf\n", amclPtr->linVel_x[0] + cgPtr->py*amclPtr->angVel_yaw[0], amclPtr->linVel_y[0] - cgPtr->px*amclPtr->angVel_yaw[0], amclPtr->angVel_yaw[0]);
	// printf("%lf, %lf, %lf\n", cgPtr->dx[0], cgPtr->dy[0], cgPtr->r[0]);
	// low pass filter
	LPfilter_1_order(cgaPtr->VX, cgaPtr->VXLPF);
	LPfilter_1_order(cgaPtr->VY, cgaPtr->VYLPF);
	LPfilter_1_order(cgaPtr->r, cgaPtr->rLPF);

	// LPfilter_2_order(amclPtr->linVel_x, amclPtr->VXLPF);
	// LPfilter_2_order(amclPtr->linVel_y, amclPtr->VYLPF);
	// LPfilter_2_order(amclPtr->angVel_yaw, amclPtr->rLPF);

	cgaPtr->AX = cgaPtr->ddx[0]*cos(cgaPtr->psi) + cgaPtr->ddy[0]*sin(cgaPtr->psi);
	cgaPtr->AY = -cgaPtr->ddx[0]*sin(cgaPtr->psi) + cgaPtr->ddy[0]*cos(cgaPtr->psi);

	cgaPtr->VX_dot = cgaPtr->AX + (cgaPtr->rLPF[0])*(cgaPtr->VYLPF[0]);
	cgaPtr->VY_dot = cgaPtr->AY - (cgaPtr->rLPF[0])*(cgaPtr->VXLPF[0]);
	
	//cgPtr->dx_r = cgPtr->VXLPF[0]+(cgPtr->rLPF[0])*(cgPtr->Y);  //X_dot
	
	// printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf\n", cgPtr->x, cgPtr->y, cgPtr->psi, cgPtr->dx[0], cgPtr->dy[0], cgPtr->r[0], cgPtr->ddx[0]);

	return 1;
}
int signal2cg(TRACKER tck, AMCLPOSE *amclPtr, CG *cgPtr, CG *cgaPtr, double t){
	tck2cg(tck, cgPtr);

	// shift data history
	dataShift(cgaPtr->dx, 5);
	dataShift(cgaPtr->dy, 5);
	dataShift(cgaPtr->r, 5);
	dataShift(cgaPtr->ddx, 5);
	dataShift(cgaPtr->ddy, 5);
	dataShift(cgaPtr->dr, 5);

	dataShift(cgaPtr->VX, 3);
	dataShift(cgaPtr->VY, 3);
	dataShift(cgaPtr->VXLPF, 3);
	dataShift(cgaPtr->VYLPF, 3);
	dataShift(cgaPtr->rLPF, 4);

	dataShift(amclPtr->x, 5);
	dataShift(amclPtr->y, 5);
	dataShift(amclPtr->yaw, 5);
	dataShift(amclPtr->linVel_x, 5);
	dataShift(amclPtr->linVel_y, 5);
	dataShift(amclPtr->angVel_yaw, 5);
	dataShift(amclPtr->ekf_vx, 5);
	dataShift(amclPtr->ekf_vy, 5);
	dataShift(amclPtr->ekf_vxl, 3);
	dataShift(amclPtr->ekf_vyl, 3);
	dataShift(amclPtr->VXLPF, 3);
	dataShift(amclPtr->VYLPF, 3);
	dataShift(amclPtr->rLPF, 4);

	amclPtr->x[0] = amclPtr->pose[0];
	amclPtr->y[0] = amclPtr->pose[1];
	amclPtr->yaw[0] = amclPtr->pose[2];

	amclPtr->ekf_vx[0] = amclPtr->ekf_v[0];
	amclPtr->ekf_vy[0] = amclPtr->ekf_v[1];

	LPfilter(amclPtr->ekf_vx, amclPtr->ekf_vxl);
	LPfilter(amclPtr->ekf_vy, amclPtr->ekf_vyl);

	differentiator_amcl(amclPtr->x, amclPtr->linVel_x);
	differentiator_amcl(amclPtr->y, amclPtr->linVel_y);
	differentiator_amcl(amclPtr->yaw, amclPtr->angVel_yaw);
	
	// get CG data
	cgaPtr->psi = amclPtr->pose[2]; // atan return -PI ~ PI
	// cgaPtr->psi = atan2(amclPtr->pose[1], amclPtr->pose[0]); // atan return -PI ~ PI

	cgaPtr->px = cgaPtr->P_BX*cos(cgaPtr->psi)-cgaPtr->P_BY*sin(cgaPtr->psi);
	cgaPtr->py = cgaPtr->P_BX*sin(cgaPtr->psi)+cgaPtr->P_BY*cos(cgaPtr->psi);

	cgaPtr->x = cgaPtr->P_BX + amclPtr->pose[0] - cgaPtr->px;
	cgaPtr->y = cgaPtr->P_BY + amclPtr->pose[1] - cgaPtr->py;

	// smoothTransition(cgPtr, cgaPtr, t);
	
	// if(t<3.0){
	// 	amclPtr->linVel_x[0] = tck.linVel[0];
	// 	amclPtr->linVel_y[0] = tck.linVel[1];
	// 	amclPtr->imu = tck.angVel[2];
	// }
	
	cgaPtr->X = cgaPtr->x*cos(cgaPtr->psi) + cgaPtr->y*sin(cgaPtr->psi);
	cgaPtr->Y = -cgaPtr->x*sin(cgaPtr->psi) + cgaPtr->y*cos(cgaPtr->psi);
	cgaPtr->r[0] = amclPtr->imu;
	cgaPtr->dx[0] = amclPtr->linVel_x[0] + cgaPtr->py*amclPtr->imu;
	cgaPtr->dy[0] = amclPtr->linVel_y[0] - cgaPtr->px*amclPtr->imu;
	cgaPtr->VX[0] = cgaPtr->dx[0]*cos(cgaPtr->psi) + cgaPtr->dy[0]*sin(cgaPtr->psi);
	cgaPtr->VY[0] = -cgaPtr->dx[0]*sin(cgaPtr->psi) + cgaPtr->dy[0]*cos(cgaPtr->psi);

	// differentiate
	differentiator(cgaPtr->dx, cgaPtr->ddx);
	differentiator(cgaPtr->dy, cgaPtr->ddy);
	differentiator(cgaPtr->r, cgaPtr->dr);

	// printf("%lf, %lf, %lf\n", amclPtr->linVel_x[0], amclPtr->linVel_y[0], amclPtr->yaw[0]);
	// printf("%lf, %lf, %lf\n", amclPtr->linVel_x[0] + cgPtr->py*amclPtr->angVel_yaw[0], amclPtr->linVel_y[0] - cgPtr->px*amclPtr->angVel_yaw[0], amclPtr->angVel_yaw[0]);
	// printf("%lf, %lf, %lf\n", cgPtr->dx[0], cgPtr->dy[0], cgPtr->r[0]);
	// low pass filter
	LPfilter_1_order(cgaPtr->VX, cgaPtr->VXLPF);
	LPfilter_1_order(cgaPtr->VY, cgaPtr->VYLPF);
	LPfilter_1_order(cgaPtr->r, cgaPtr->rLPF);

	// LPfilter_2_order(amclPtr->linVel_x, amclPtr->VXLPF);
	// LPfilter_2_order(amclPtr->linVel_y, amclPtr->VYLPF);
	// LPfilter_2_order(amclPtr->angVel_yaw, amclPtr->rLPF);

	cgaPtr->AX = cgaPtr->ddx[0]*cos(cgaPtr->psi) + cgaPtr->ddy[0]*sin(cgaPtr->psi);
	cgaPtr->AY = -cgaPtr->ddx[0]*sin(cgaPtr->psi) + cgaPtr->ddy[0]*cos(cgaPtr->psi);

	cgaPtr->VX_dot = cgaPtr->AX + (cgaPtr->rLPF[0])*(cgaPtr->VYLPF[0]);
	cgaPtr->VY_dot = cgaPtr->AY - (cgaPtr->rLPF[0])*(cgaPtr->VXLPF[0]);
	
	//cgPtr->dx_r = cgPtr->VXLPF[0]+(cgPtr->rLPF[0])*(cgPtr->Y);  //X_dot
	
	// printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf\n", cgPtr->x, cgPtr->y, cgPtr->psi, cgPtr->dx[0], cgPtr->dy[0], cgPtr->r[0], cgPtr->ddx[0]);

	return 1;
	
}

double sigmoid(double x, double k){
	return 1.0 / (1.0 + exp(-k * (x - 3.0)));
}

void smoothTransition(CG *cgPtr, CG *cgaPtr, double t){
	double transitionProgress = t / T_TIME;
	double sigmoidValue = sigmoid(transitionProgress, 5.0);

	cgaPtr->x = (1.0 - sigmoidValue) * cgPtr->x + sigmoidValue * cgaPtr->x;
	cgaPtr->y = (1.0 - sigmoidValue) * cgPtr->y + sigmoidValue * cgaPtr->y;
	cgaPtr->psi = (1.0 - sigmoidValue) * cgPtr->psi + sigmoidValue * cgaPtr->psi;
}


//========================================================================
//                               cg2tires
//========================================================================
int cg2tires(CG cg, TIRES *tPtr)
{
	for(int i=0; i<4; i++){
		tPtr->theTire[i].VWX = cg.VXLPF[0] + tPtr->s[i]*cg.rLPF[0];
		tPtr->theTire[i].VWY = cg.VYLPF[0] + tPtr->l[i]*cg.rLPF[0];

		if( (fabs(tPtr->theTire[i].VWX) > 0.05) || (fabs(tPtr->theTire[i].VWY) > 0.05) ){
			tPtr->theTire[i].alpha = atan2(tPtr->theTire[i].VWY, tPtr->theTire[i].VWX);
		}
		else{
			tPtr->theTire[i].alpha = 0.0;
		}

		if( (fabs(tPtr->theTire[i].VWX) > 0.1) || (fabs(tPtr->theTire[i].omegaLPF[0]) > 1.0) ){
			tPtr->theTire[i].lambda = ((tPtr->R)*(tPtr->theTire[i].omegaLPF[0]) - tPtr->theTire[i].VWX)
			              / fmax(fabs((tPtr->R)*(tPtr->theTire[i].omegaLPF[0])), fabs(tPtr->theTire[i].VWX));
		}
		else{
			tPtr->theTire[i].lambda = 0.0;
		}
	}
	return 1;
}






