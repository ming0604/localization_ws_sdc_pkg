#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "upperLayer.h"
#include "parameters.h"

inline double mySq(double x){return x*x;}
inline double tanh(double x){return (exp(2.0*x)-1.0)/(exp(2.0*x)+1.0);}

void trapezoidalSpeed(double t, DESIRED_TRAJECTORY *d, double L, double v, double t1)
{
	static double a, t2 , t3;
	a = v/t1;
	t2 = L/v;
	t3 = t1+t2;

	if(t<t1){
		d->xd = 0.5*a*t*t;
		d->dxd = a*t;
		d->ddxd = a;
	}
	else if(t<t2){
		d->xd = (t - 0.5*t1)*v;
		d->dxd = v;
		d->ddxd = 0.0;
	}
	else if(t<t3){
		d->xd = t*v - 0.5*a*t1*t1 - 0.5*a*(t-t2)*(t-t2);
		d->dxd = v - a*(t-t2);
		d->ddxd = -a;
	}
	else{
		d->xd = L;
		d->dxd = 0.0;
		d->ddxd = 0.0;
	}
	d->dddxd = 0.0;
}

// ================================================================
//                       1.  yd = atan(xd)
// ================================================================
int genDesiredTrajectory1(double t, UPPERLAYER_DATA *upperLayerDataPtr)
{
	// create shorter pointer for better code reading
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);

	// mid calculation
	double Den, dDen, ddDen; // Den: denominator of d(atan)/dt


	// avoid numerical problem
	if (t-0.0 < 0.001) t=0.001;
	if (10.0-t < 0.001) t=9.999;

	// generate xd, dxd, ddxd, dddxd
	if(t<=2){
		d->xd = 0.15625*t*t;
		d->dxd = 0.3125*t;
		d->ddxd = 0.3125;
	}
	else if(t<=8){
		d->xd = 0.625*t-0.625;
		d->dxd = 0.625;
		d->ddxd = 0.0;
	}
	else if(t<10.0){
		d->xd = -0.15625*t*t+3.125*t-10.625;
		d->dxd = -0.3125*t+3.125;
		d->ddxd = -0.3125;
	}
	else{
		d->xd = 0.0;
		d->dxd = 0.0;
		d->ddxd = 0.0;
	}
	d->dddxd = 0.0;

	// generate yd, dyd, ddyd, dddyd
	Den = (d->xd)*(d->xd) - 5*(d->xd) + 7.25;
	dDen = (2*(d->xd) - 5)*(d->dxd);
	ddDen = 2*(d->dxd)*(d->dxd) + (2*(d->xd) - 5)*(d->ddxd);
	d->yd = 0.4201*atan(d->xd - 2.5) + 0.5;
	d->dyd = 0.4201/Den*(d->dxd);
	d->ddyd = 0.4201*((d->ddxd)*Den - (d->dxd)*dDen)/(Den*Den);
	d->dddyd = 0.4201*(-(d->dxd)*ddDen*Den - ((d->ddxd)*Den - (d->dxd)*dDen)*2*dDen)/(Den*Den*Den);

	d->psid = atan2(d->dyd,d->dxd);
    d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
    d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));

	return 1;
}

// ================================================================
//                 2.  yd = a*atan(b*(xd-L/2)) + W/2
// ================================================================
int genDesiredTrajectory2(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double W, double b, double v)
{
	// input parameters: L=>xd displace, W=>yd displace, b=>swifting rate, v=>x velocity
	// xd: 0 => L

	// create shorter pointer for better code reading
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);

	static const double a = W*0.5/atan(b*L*0.5);
	static const double t_end = L/v;
	double Den, dDen, ddDen;

	if(t < L/v){
		d->xd = v*t;
		d->dxd = v;
		d->ddxd = 0.0;
		d->dddxd = 0.0;

		// generate yd, dyd, ddyd, dddyd
		Den = 1.0 + b*b*mySq(d->xd - L*0.5);
		dDen = 2*b*b*(d->xd - L*0.5)*(d->dxd);
		ddDen = 2*b*b*mySq(d->dxd);
		d->yd = a*atan(b*(d->xd - L*0.5)) + W*0.5;
		d->dyd = a/Den*b*(d->dxd);
		d->ddyd = a*(-b*(d->dxd)*dDen)/mySq(Den);
		d->dddyd = a*b*(d->dxd)*(2*mySq(dDen) - ddDen*Den);

		d->psid = atan2(d->dyd,d->dxd);
		d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
		d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));

	}
	else{
		d->xd = L;
		d->dxd = 0.0;
		d->ddxd = 0.0;
		d->dddxd = 0.0;
		d->yd = W;
		d->dyd = 0.0;
		d->ddyd = 0.0;
		d->dddyd = 0.0;
		d->rd = 0.0;
		d->drd = 0.0;
	}

	return 1;
}

// ================================================================
//                       3.  straight line
// ================================================================
int genDesiredTrajectory3(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double v, double t1)
{
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);

	trapezoidalSpeed(t, d, L, v, t1); // generate xd, dxd, ddxd, dddxd

	d->yd = 0.0;
	d->dyd = 0.0;
	d->ddyd = 0.0;
	d->dddyd = 0.0;
	d->psid = 0.0;
	d->rd = 0.0;
	d->drd = 0.0;

	return 1;
}

// ================================================================
//                 4.  yd = a*tanh(b*(xd-L/2)) + W/2
// ================================================================
int genDesiredTrajectory4(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double W, double b, double v, double t1)
{
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);
	static const double a = W*0.5/tanh(b*L*0.5);
	static double H, dH, ddH;
	static double t3 = L/v + t1;

	trapezoidalSpeed(t, d, L, v, t1); // generate xd, dxd, ddxd, dddxd

	H = tanh(b*(d->xd - 0.5*L)); // dH/dt = (1-H^2)*b*dx/dt
	dH = (1.0 - H*H)*b*(d->dxd);
	ddH = b*(-2.0*H*dH*(d->dxd) + (1.0 - H*H)*(d->ddxd));

	d->yd = a*H + W*0.5;
	d->dyd = a*dH;
	d->ddyd = a*ddH;
	d->dddyd = a*b*(-2.0*(dH*dH*(d->dxd) + H*ddH*(d->dxd) + H*dH*(d->ddxd)) - 2.0*H*dH*(d->ddxd) + (1.0 - H*H)*(d->dddxd));
	if (t<t3){
		d->psid = atan2(d->dyd,d->dxd);
	} // keep psid after end of trajectory, avoiding sudden drop

	if( (d->dxd)!=0.0 || (d->dyd)!=0.0 ){
		d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
		d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));
	}
	else{
		d->rd = 0.0;
		d->drd = 0.0;
	}

	return 1;
}

// ================================================================
//                 5.  decreasing radius turn
// ================================================================
int genDesiredTrajectory5(double t, UPPERLAYER_DATA *upperLayerDataPtr, double v, const bool B) // B=true -> left turn, B=false -> right turn
{
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);
	static const double r0 = 5.0;
	static double t4turning, u, r; // u from 0 -> 1 means yaw whole around the centor
	static double a = v/sqrt(36.0+4*PI*PI*r0*r0); // a = du/dt
	static double tend = 0.2/a + 1.5; // total length is decided here. tend = u_end/a + 1.5
	static double b = (double)B;
	static double v_income = sqrt(mySq(r0*2.0*PI*a) + mySq(6.0*a));
	static double l, w, angle_income_tan, angle_income;
	static double xd_rotated, dxd_rotated, ddxd_rotated, yd_rotated, dyd_rotated, ddyd_rotated;

	static bool   firstRound = true;

	t4turning = t-1.5;

	if (firstRound){
		if (B){
			angle_income_tan = 6.0*a/(r0*2.0*PI*a);  //r(0)=5
		}
		else{
			angle_income_tan = -6.0*a/(r0*2.0*PI*a);  //r(0)=5
		}
		angle_income = atan(angle_income_tan);
		l = (0.5*v_income*1 + v_income*0.5)*cos(angle_income);
		w = l*angle_income_tan;

		printf("starting angle = %f(rad) or %f(degree)\n", angle_income, angle_income*180.0/PI);
		printf("tend: %f\n", tend);

		firstRound = false;
	}

	if (t<1.5){   // accelerate to constant speed
		trapezoidalSpeed(t, d, l+2, v_income*cos(angle_income), 1); // generate xd, dxd, ddxd, dddxd
		d->yd = d->xd*angle_income_tan;
		d->dyd = d->dxd*angle_income_tan;
		d->ddyd = d->ddxd*angle_income_tan;
		d->dddyd = 0.0;
		d->psid = angle_income;
		d->rd = 0.0;
		d->drd = 0.0;
	}
	else if (t<tend){  // decreasing radius of curvature
		u = a*t4turning;
		r = r0 - 6*u; //radius
		d->xd = r*sin(2*PI*u) + l;
		if (B){ //left turn
			d->yd = r*cos(2*PI*u + PI) + r0 + w; //r(0)=5
		}
		else{ //right turn
			d->yd = r*cos(2*PI*u) - r0 + w; //r(0)=5
		}
		d->dxd = -6.0*a*sin(2*PI*u) + r*cos(2*PI*u)*2*PI*a;
		d->dyd = -6.0*a*cos(2*PI*u+PI*b) - r*sin(2*PI*u+PI*b)*2*PI*a;
		d->ddxd = -36.0*PI*a*a*cos(2*PI*u) - 4*PI*PI*a*a*r*sin(2*PI*u);
		d->ddyd = 36.0*PI*a*a*sin(2*PI*u+PI*b) - 4*PI*PI*a*a*r*cos(2*PI*u+PI*b);
		d->dddxd = 86.0*PI*PI*a*a*a*sin(2*PI*u) - 8*PI*PI*PI*a*a*a*r*cos(2*PI*u);
		d->dddyd = 86.0*PI*PI*a*a*a*cos(2*PI*u+PI*b) + 8*PI*PI*PI*a*a*a*r*sin(2*PI*u+PI*b);
		d->psid = atan2(d->dyd,d->dxd);
		d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
		d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));
	}
	else{
		//d->xd = l;
		d->dxd = 0.0;
		d->ddxd = 0.0;
		d->dddxd = 0.0;
		/*
		if (B){ //left turn
			d->yd = 4.0;
		}
		else{ //right turn
			d->yd = -4.0;
		}
		*/
		d->dyd = 0.0;
		d->ddyd = 0.0;
		d->dddyd = 0.0;
		d->rd = 0.0;
		d->drd = 0.0;
	}

	if(t<tend){
		// rotate angle_income
		xd_rotated = (d->xd)*cos(angle_income) + (d->yd)*sin(angle_income);
		yd_rotated = -(d->xd)*sin(angle_income) + (d->yd)*cos(angle_income);
		dxd_rotated = (d->dxd)*cos(angle_income) + (d->dyd)*sin(angle_income);
		dyd_rotated = -(d->dxd)*sin(angle_income) + (d->dyd)*cos(angle_income);
		ddxd_rotated = (d->ddxd)*cos(angle_income) + (d->ddyd)*sin(angle_income);
		ddyd_rotated = -(d->ddxd)*sin(angle_income) + (d->ddyd)*cos(angle_income);

		// over write
		d->xd = xd_rotated;
		d->yd = yd_rotated;
		d->dxd = dxd_rotated;
		d->dyd = dyd_rotated;
		d->ddxd = ddxd_rotated;
		d->ddyd = ddyd_rotated;
		d->psid -= angle_income;
	}

	return 1;
}
// ================================================================
//                 6. forward to v=0.5 and single lane change  yd = a*tanh(b*(xd-L/2)) + W/2
// ================================================================
int genDesiredTrajectory6(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L, double W, double b, double v)
{
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);
	static const double a = W*0.5/tanh(b*L*0.5);
	static double H, dH, ddH;

    static double vth = 0.7;
    static double t0 = 0.5;
    static double t01 = 1.5;
	//static double t3 = L/v + t1;
	static double a0,a1,t1,t2 , t3, t4;
	static double x_move;

	x_move = vth*t0*0.5 + vth*(t01-t0);
    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+1;
	t4 = t3+t0;

	a0 = vth/t0;
	a1 = (v-vth)/(t1-t01);

	if(t<t0){
		d->xd = 0.5*a0*t*t;
		d->dxd = a0*t;
		d->ddxd = a0;
	}
	else if(t<t01){
		d->xd = 0.5*a0*t0*t0+vth*(t-t0);
		d->dxd = vth;
		d->ddxd = 0;		

	}
	else if(t<t1){
        d->dxd = vth+a1*(t-t01);
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+d->dxd)*(t-t01)/2;
		d->ddxd = a1;
	}
	else if(t<t2){
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t-t1);
		d->dxd = v ;
		d->ddxd = 0;
	}
	else if(t<t3){
        d->dxd = v-a1*(t-t2);
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1) + (v+d->dxd)*(t-t2)/2;
		d->ddxd = -a1;
	}
	else if (t<t4){
        d->dxd = vth-a0*(t-t3);
		d->xd =0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1) + (v+vth)*(t3-t2)/2 +(vth+d->dxd)*(t-t3)/2;
		d->ddxd = -a0;

	}
	else{

		d->xd =0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1) + (v+vth)*(t3-t2)/2 +(vth)*(t4-t3)/2;
		d->dxd = 0;
		d->ddxd = 0;
	}
	d->dddxd = 0.0;
	//trapezoidalSpeed(t, d, L, v, t1); // generate xd, dxd, ddxd, dddxd

    if (t<t01){
        d->yd = 0;
        d->dyd = 0;
        d->ddyd = 0;
        d->dddyd = 0;

    }
    else if(t<t3){
        H = tanh(b*(d->xd - 0.5*L-x_move)); // dH/dt = (1-H^2)*b*dx/dt
        dH = (1.0 - H*H)*b*(d->dxd);
        ddH = b*(-2.0*H*dH*(d->dxd) + (1.0 - H*H)*(d->ddxd));

        d->yd = a*H + W*0.5;
        d->dyd = a*dH;
        d->ddyd = a*ddH;
        d->dddyd = a*b*(-2.0*(dH*dH*(d->dxd) + H*ddH*(d->dxd) + H*dH*(d->ddxd)) - 2.0*H*dH*(d->ddxd) + (1.0 - H*H)*(d->dddxd));

    }
    else{
    	double x = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1) + (v+vth)*(t3-t2)/2;

    	H = tanh(b*(x - 0.5*L-x_move));
        d->yd = a*H + W*0.5;
        d->dyd = 0;
        d->ddyd = 0;
        d->dddyd = 0;
    }

	if (t<t4){
		d->psid = atan2(d->dyd,d->dxd);
	} // keep psid after end of trajectory, avoiding sudden drop

	if( (d->dxd)!=0.0 || (d->dyd)!=0.0 ){
		d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
		d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));
	}
	else{
		d->rd = 0.0;
		d->drd = 0.0;
	}

	return 1;
}

// ================================================================
//                 7. straight line,  forward to v=0.7 and use controller to v
// ================================================================
int genDesiredTrajectory7(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L,double v)
{
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);
	
	

    static double vth = 0.7;
    static double t0 = 0.5;
    static double t01 = 1.5;
	//static double t3 = L/v + t1;
	static double a0,a1,t1,t2 , t3, t4;

    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+1;
	t4 = t3+t0;

	a0 = vth/t0;
	a1 = (v-vth)/(t1-t01);

	if(t<t0){
		d->xd = 0.5*a0*t*t;
		d->dxd = a0*t;
		d->ddxd = a0;
	}
	else if(t<t01){
		d->xd = 0.5*a0*t0*t0+vth*(t-t0);
		d->dxd = vth;
		d->ddxd = 0;		

	}
	else if(t<t1){
        d->dxd = vth+a1*(t-t01);
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+d->dxd)*(t-t01)/2;
		d->ddxd = a1;
	}
	else if(t<t2){
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t-t1);
		d->dxd = v ;
		d->ddxd = 0;
	}
	else if(t<t3){
        d->dxd = v-a1*(t-t2);
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1) + (v+d->dxd)*(t-t2)/2;
		d->ddxd = -a1;
	}
	else if (t<t4){
        d->dxd = vth-a0*(t-t3);
		d->xd =0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1) + (v+vth)*(t3-t2)/2 +(vth+d->dxd)*(t-t3)/2;
		d->ddxd = -a0;

	}
	else{

		d->xd =0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1) + (v+vth)*(t3-t2)/2 +(vth)*(t4-t3)/2;
		d->dxd = 0;
		d->ddxd = 0;
	}
	d->dddxd = 0.0;
	//trapezoidalSpeed(t, d, L, v, t1); // generate xd, dxd, ddxd, dddxd

    if (t<t01){
        d->yd = 0;
        d->dyd = 0;
        d->ddyd = 0;
        d->dddyd = 0;

    }
    else if(t<t3){
        d->yd = 0;
        d->dyd = 0;
        d->ddyd = 0;
        d->dddyd = 0;
    }
    else{
        d->yd = 0;
        d->dyd = 0;
        d->ddyd = 0;
        d->dddyd = 0;
    }

	if (t<t4){
		d->psid = atan2(d->dyd,d->dxd);
	} // keep psid after end of trajectory, avoiding sudden drop

	if( (d->dxd)!=0.0 || (d->dyd)!=0.0 ){
		d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
		d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));
	}
	else{
		d->rd = 0.0;
		d->drd = 0.0;
	}

	return 1;
}


// ================================================================
//                 8. straight line and circle
// ================================================================
int genDesiredTrajectory8(double t, UPPERLAYER_DATA *upperLayerDataPtr, double L,double v,double R)
{
	static DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);
	
	

    static double vth = 0.7;
    static double t0 = 0.5;
    static double t01 = 1.5;
	//static double t3 = L/v + t1;
	static double a0,a1,t1,t2,t3,t4,t5;
	double T;
  	double w;
  	static double x_move;
  	w = v/R;
  	T = 2*PI/w;

    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+0.25*T;
	t4 = t3+1;
	t5 = t4+t0;

	a0 = vth/t0;
	a1 = (v-vth)/(t1-t01);
  	x_move = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1);

	if(t<t0){
		d->xd = 0.5*a0*t*t;
		d->dxd = a0*t;
		d->ddxd = a0;
		d->dddxd = 0;

		d->yd = 0;
		d->dyd = 0;
		d->ddyd = 0;
		d->dddyd = 0;

		d->psid = 0;
		d->rd = 0;
		d->drd = 0;

	}
	else if(t<t01){
		d->xd = 0.5*a0*t0*t0+vth*(t-t0);
		d->dxd = vth;
		d->ddxd = 0;	
		d->dddxd = 0;

		d->yd = 0;
		d->dyd = 0;
		d->ddyd = 0;
		d->dddyd = 0;

		d->psid = 0;
		d->rd = 0;
		d->drd = 0;	

	}
	else if(t<t1){
        d->dxd = vth+a1*(t-t01);
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+d->dxd)*(t-t01)/2;
		d->ddxd = a1;
		d->dddxd = 0;

		d->yd = 0;
		d->dyd = 0;
		d->ddyd = 0;
		d->dddyd = 0;

		d->psid = 0;
		d->rd = 0;
		d->drd = 0;
	}
	else if(t<t2){
		d->xd = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t-t1);
		d->dxd = v ;
		d->ddxd = 0;
		d->dddxd = 0;

		d->yd = 0;
		d->dyd = 0;
		d->ddyd = 0;
		d->dddyd = 0;

		d->psid = 0;
		d->rd = 0;
		d->drd = 0;
	}
	else if(t<t3){
		d->xd = x_move + R*cos(w*(t-t2+0.75*T));
		d->dxd = -R*w*sin(w*(t-t2+0.75*T)) ;
		d->ddxd = -R*w*w*cos(w*(t-t2+0.75*T));
		d->dddxd = R*w*w*w*sin(w*(t-t2+0.75*T)) ;

		d->yd = R+R*sin(w*(t-t2+0.75*T));
		d->dyd = R*w*cos(w*(t-t2+0.75*T)) ;
		d->ddyd = -R*w*w*sin(w*(t-t2+0.75*T));;
		d->dddyd = -R*w*w*w*cos(w*(t-t2+0.75*T)) ;

		d->psid = atan2(d->dyd,d->dxd);
		if( (d->dxd)!=0.0 || (d->dyd)!=0.0 ){
			d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
			d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));
		}
		else{
			d->rd = 0.0;
			d->drd = 0.0;
		}
	
	}
	else if (t<t4){

		d->xd =x_move + R*cos(w*(t3-t2+0.75*T));
        d->dxd = 0;
		d->ddxd = 0;
		d->dddxd = 0;

		d->dyd = R*w*cos(w*(t3-t2+0.75*T)) -a1*(t-t3);
		d->yd = R+R*sin(w*(t3-t2+0.75*T)) + (v+d->dyd)*(t-t3)/2;
		d->ddyd = -a1;
		d->dddyd = 0;

		d->psid = PI/2;
		if( (d->dxd)!=0.0 || (d->dyd)!=0.0 ){
			d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
			d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));
		}
		else{
			d->rd = 0.0;
			d->drd = 0.0;
		}

	}
	else if(t<t5){
		d->xd =x_move + R*cos(w*(t3-t2+0.75*T));
        d->dxd = 0;
		d->ddxd = 0;
		d->dddxd = 0;

		d->dyd = vth - a0*(t-t4);
		d->yd = R+R*sin(w*(t3-t2+0.75*T)) + (v+vth)*(t4-t3)/2 + (vth+d->dyd)*(t-t4)/2;
		d->ddyd = -a0;
		d->dddyd = 0;

		d->psid = PI/2;
		if( (d->dxd)!=0.0 || (d->dyd)!=0.0 ){
			d->rd = ((d->ddyd)*(d->dxd) - (d->dyd)*(d->ddxd))/((d->dxd)*(d->dxd) + (d->dyd)*(d->dyd));
			d->drd = (((d->dddyd)*(d->dxd)-(d->dyd)*(d->dddxd))*((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd)) - ((d->ddyd)*(d->dxd)-(d->dyd)*(d->ddxd))*(2*(d->dxd)*(d->ddxd)+2*(d->dyd)*(d->ddyd))) / mySq((d->dxd)*(d->dxd)+(d->dyd)*(d->dyd));
		}
		else{
			d->rd = 0.0;
			d->drd = 0.0;
		}

	}
	else{
		d->xd =x_move + R*cos(w*(t3-t2+0.75*T));
        d->dxd = 0;
		d->ddxd = 0;
		d->dddxd = 0;

		d->dyd = 0;
		d->yd = R+R*sin(w*(t3-t2+0.75*T)) + (v+vth)*(t4-t3)/2 + vth*(t5-t4)/2;
		d->ddyd = 0;
		d->dddyd = 0;		

		d->psid = PI/2;
		d->rd = 0.0;
		d->drd = 0.0;
		

	}


	return 1;
}
/*
// ================================================================
//                        preview trajectory
// ================================================================
int genPreviewTrajectory(UPPERLAYER_DATA *upperLayerDataPtr, CG *c)
{
	// create shorter pointer for better code reading
	DESIRED_TRAJECTORY *d = &(upperLayerDataPtr->desiredTrajectory);
	PREVIEW_TRAJECTORY *p = &(upperLayerDataPtr->previewTrajectory);

	p->xp = (d->xd) - (p->P)*cos(d->psid);
	p->yp = (d->yd) - (p->P)*sin(d->psid);
	p->dxp = (d->dxd) + (p->P)*sin(d->psid)*(d->rd);
	p->dyp = (d->dyd) - (p->P)*cos(d->psid)*(d->rd);
	p->ddxp = (d->dxd) + (p->P)*cos(d->psid)*(d->rd)*(d->rd) + (p->P)*sin(d->psid)*(d->drd);
	p->ddyp = (d->dyd) + (p->P)*sin(d->psid)*(d->rd)*(d->rd) - (p->P)*cos(d->psid)*(d->drd);

	// transformation (rotional)
	p->Xp = (p->xp)*cos(c->psi) + (p->yp)*sin(c->psi);
	p->Yp = -(p->xp)*sin(c->psi) + (p->yp)*cos(c->psi);
	p->VXp = (p->dxp)*cos(c->psi) + (p->dyp)*sin(c->psi);
	p->VYp = -(p->dxp)*sin(c->psi) + (p->dyp)*cos(c->psi);
	p->AXp = (p->ddxp)*cos(c->psi) + (p->ddyp)*sin(c->psi);
	p->AYp = -(p->ddxp)*sin(c->psi) + (p->ddyp)*cos(c->psi);

	return 1;
}
*/
// ================================================================
//                      saturation function
// ================================================================
double saturation(double in, double epsilon)
{
	if (epsilon < 0.0){epsilon = -epsilon;}
	if (epsilon > 0.0){
		if(in > epsilon) return 1.0;
		else if(in < -epsilon) return -1.0;
		else return in/epsilon;
	}
	else{ // epsilon is 0 => equivalent to sign function
		if(in==0.0) return 0.0;
		else return (in > 0.0) ? 1.0 : -1.0;
	}
}

double saturation(double in, double epsilonP, double epsilonN)
{
	if (epsilonP < 0.0){epsilonP = -epsilonP;}
	if (epsilonN < 0.0){epsilonN = -epsilonN;}

	if(in > epsilonP) return 1.0;
	else if(in < -epsilonN) return -1.0;
	else{
		if(in == 0.0) return 0.0;
		if(in > 0.0) return in/epsilonP;
		if(in < 0.0) return in/epsilonN;

	}
}
double sgn(double in){
    if(in == 0.0) return 0.0;
    if(in > 0.0) return 1.0;
    if(in < 0.0) return -1.0;
}
// ================================================================
//                      derivative to get dy_r from y_r
// ================================================================
void derivative(double *x, double *y)
{

	y[0] = 95.92*x[0] - 95.92*x[1]+0.04076*y[1] ;

}
// ================================================================
//                     sqrt3
// ================================================================
double sqrt3(double a){
    if (a>=0){
        return pow(a,1.0/3);
    }
    else{

        return -pow(-a,1.0/3);
    }


}
// ================================================================
//                      calculate_road_error_genDesiredTrajectory6
// ================================================================

int calculate_road_error(CG cg, UPPERLAYER_DATA *u,double sec,double v){

    static double L,W,b,a;
    static double x,dx,ddx,dddx;
    static double H,dH,ddH,y,dy,ddy,dddy;
    static double f,f_dot,error;
    static double P,Q;
    static double y_r;

    static double vth; 
    static double t0;
    static double t01;
    static double t;

	static double a0,a1,t1,t2 , t3, t4;
	static double x_move;
	double rd,drd,ddyd_r,ddy_real;
	double dyd_r,dy_real;
	double L1,L2;
	double cox,coy;
	double COX,COY;
	double j_dir;
	double psid;

	//judge direction
	cox = 0;
    coy = 1;
    ////
	vth = 0.7;
	t0 = 0.5;

    L=7;
    W= 1.17;
    b= 0.6;
/*
    L=5;
    W= 0.7;
    b= 1;

*/
	a0 = vth/t0;
	a1 = (v-vth)/(t1-t01);

	t01 = t0+1;
    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+1;
	t4 = t3+t0;

	x_move = vth*t0*0.5 + vth*(t01-t0);

	//t=sec;

    /*
    L=7.0;
    W=1.17;
    b=0.6;
    v=1.5;
    t1=1.0;
    */
    a=W*0.5/tanh(b*L/2);



    //printf("%f\n",acc);

    
/*
    if(t<=0){
        t = 0.01;
    }
*/
    static double t_new;
    static double f_ans;
    static int ccount = 1;
    P = cg.x;
    Q = cg.y;

    if(sec<t01){


        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);
        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_LPF, 3);
        //dataShift(u->ddy_r, 2);
        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);



        y_r = Q-0;
        u->ccount = ccount;
        u->c_x = P;
        u->c_y = 0;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-0;
        u->r_r[0] = cg.rLPF[0] - 0;

        ///calculate dy_r
        dy_real = cg.dy[0];
        dyd_r = 0;
        u->dy_r[0] = dy_real - dyd_r;

        //derivative(u->y_r,u->dy_r);
        //derivative(u->dy_r,u->ddy_r);
        //derivative(u->psi_r,u->r_r);   
        LPfilter(u->y_r, u->y_r_LPF);   
        LPfilter(u->dy_r, u->dy_r_LPF);
        LPfilter(u->psi_r, u->psi_r_LPF);
        //LPfilter(u->r_r, u->r_r_LPF);


        printf("t=%f,P=%f,Q=%f,error=%f,y_r=%f,dy_r=%f,psi_r=%f, r_r=%f\n",sec,P,Q,error,u->y_r[0],u->dy_r_LPF[0],u->psi_r[0],u->r_r[0]);
        return 1;
    }
    else if(sec<t3){

        printf("calculate road error\n");
        t=sec;

        while(1){

            if(t<t1){
                dx = vth+a1*(t-t01);
                x = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+dx)*(t-t01)/2;
                ddx = a1;
                dddx = 0;

                H = tanh(b*(x-0.5*L-x_move));   //0.625 is the distance of linear acceleration
                dH = (1-H*H)*b*dx;
                ddH = b*(-2*H*dH*dx + (1-H*H)*ddx);

                y = a*H+0.5*W;
                dy = a*dH;
                ddy = a*ddH;
                dddy = a*b*(-2.0*(dH*dH*(dx) + H*ddH*(dx) + H*dH*(ddx)) - 2.0*H*dH*(ddx) + (1.0 - H*H)*(dddx));
            }
            else if(t<t2){
                x= 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 +v*(t-t1);
                dx = v ;
                ddx = 0;
                dddx = 0;

                H = tanh(b*(x-0.5*L-x_move));
                dH = (1-H*H)*b*dx;
                ddH = b*(-2*H*dH*dx + (1-H*H)*ddx);

                y = a*H+0.5*W;
                dy = a*dH;
                ddy = a*ddH;
                dddy = a*b*(-2.0*(dH*dH*(dx) + H*ddH*(dx) + H*dH*(ddx)) - 2.0*H*dH*(ddx) + (1.0 - H*H)*(dddx));
            }
            else if(t<t3){
                dx = v-a1*(t-t2);
                x = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 +v*(t2-t1)+(v+dx)*(t-t2)/2;
                ddx = -a1;
                dddx = 0;

                H = tanh(b*(x-0.5*L-x_move));
                dH = (1-H*H)*b*dx;
                ddH = b*(-2*H*dH*dx + (1-H*H)*ddx);

                y = a*H+0.5*W;
                dy = a*dH;
                ddy = a*ddH;
                dddy = a*b*(-2.0*(dH*dH*(dx) + H*ddH*(dx) + H*dH*(ddx)) - 2.0*H*dH*(ddx) + (1.0 - H*H)*(dddx));
            }
            L1=sqrt((P-x)*(P-x)+(Q-y)*(Q-y));
            L2=sqrt(dx*dx+dy*dy);
            f=((P-x)*dx+(Q-y)*dy)/(L1*L2);
            f_dot =(-dx*dx+(P-x)*ddx - dy*dy+(Q-y)*ddy)/(L1*L2);
            f_ans = f/f_dot;

            t_new = t - f/f_dot;

            error = fabs(t_new - t);
            //error<0.00001 || t==0.5
            if(error<0.00001){


                printf("find  count=%d\n",ccount);
                //printf("count = %d,x=%f,y=%f,error=%f\n",count,x,y,error);
                dataShift(u->y_r, 3);
                dataShift(u->y_r_LPF, 3);
                dataShift(u->dy_r, 3);
                dataShift(u->dy_r_LPF, 3);
                dataShift(u->ddy_r, 3);
                dataShift(u->ddy_r_LPF, 3);
                //dataShift(u->ddy_r, 2);
        		dataShift(u->psi_r, 3);
        		dataShift(u->psi_r_LPF, 3);
                dataShift(u->r_r, 3);

                


                ///calculate psid rd drd
                psid = atan2(dy,dx);
				if( (dx)!=0.0 || (dy)!=0.0 ){
					
					rd = (ddy*dx-dy*ddx)/(dx*dx+dy*dy);
					drd = (((dddy)*(dx)-(dy)*(dddx))*((dx)*(dx)+(dy)*(dy)) - ((ddy)*(dx)-(dy)*(ddx))*(2*(dx)*(ddx)+2*(dy)*(ddy))) / mySq((dx)*(dx)+(dy)*(dy));
				}
				else{
					rd = 0.0;
					drd = 0.0;
				}

       
                ///calculate dy_r
                dy_real = -cg.dx[0]*sin(psid)+cg.dy[0]*cos(psid)-rd*(cg.x*cos(psid)+cg.y*sin(psid));
                dyd_r = -dx*sin(psid)+dy*cos(psid)-rd*(x*cos(psid)+y*sin(psid));
				u->dy_r[0] =dy_real - dyd_r; 
                ///calculate ddy_r
                ddy_real = -cg.ddx[0]*sin(psid)+cg.ddy[0]*cos(psid)-2*rd*(cg.dx[0]*cos(psid)+cg.dy[0]*sin(psid))-rd*rd*(-cg.x*sin(psid)+cg.y*cos(psid))-drd*(cg.x*cos(psid)+cg.y*sin(psid));
                ddyd_r = -ddx*sin(psid)+ddy*cos(psid)-2*rd*(dx*cos(psid)+dy*sin(psid))-rd*rd*(-x*sin(psid)+y*cos(psid))-drd*(x*cos(psid)+y*sin(psid));
                u->ddy_r[0] = ddy_real-ddyd_r;

                //judge direction of road error
            	COX = cox*cos(atan2(dy,dx))+coy*sin(atan2(dy,dx));
            	COY = -cox*sin(atan2(dy,dx))+coy*cos(atan2(dy,dx));
            	j_dir = COX*(P-x)+COY*(Q-y);

                y_r = sqrt((x-P)*(x-P)+(y-Q)*(y-Q));
                if(j_dir>=0){
                	y_r = y_r;
                }
                else{
                	y_r = -y_r;
                }
                ////////////////////

                u->ccount = ccount;
                u->c_x = x;
                u->c_y = y;
                u->y_r[0] = y_r;
                u->psi_r[0] = cg.psi - psid;
                u->r_r[0]=cg.rLPF[0]-rd;
                //derivative(u->y_r,u->dy_r);
                //derivative(u->dy_r,u->ddy_r);
                //derivative(u->psi_r,u->r_r);
                LPfilter(u->y_r, u->y_r_LPF); 
                LPfilter(u->dy_r, u->dy_r_LPF);
                //LPfilter(u->ddy_r, u->ddy_r_LPF);
                u->ddy_r_LPF[0] = u->ddy_r[0];
                LPfilter(u->psi_r, u->psi_r_LPF);
                // printf("sec = %f,t_new=%f,t=%f,x=%f,y=%f,P=%f,Q=%f,y_r=%f,dy_r=%f,psi_r=%f, r_r=%f\n",sec,t_new,t,x,y,P,Q,u->y_r[0],u->dy_r_LPF[0],u->psi_r[0],u->r_r_LPF[0]);
                // printf("dx=%f,dy=%f\n",dx,dy);
                // printf("0.5*a0*t0*t0=%f, (vth+dx)*(t-t0)/2=%f\n",0.5*a0*t0*t0,(vth+dx)*(t-t0)/2);
                //return 1;
                break;
            }

            else if(ccount > 8){

                printf("over 8 times\n");


                dataShift(u->y_r, 3);
                dataShift(u->y_r_LPF, 3);
                dataShift(u->dy_r, 3);
                dataShift(u->dy_r_LPF, 3);
                dataShift(u->ddy_r, 3);
                dataShift(u->ddy_r_LPF, 3);
                //dataShift(u->ddy_r, 2);
		        dataShift(u->psi_r, 3);
		        dataShift(u->psi_r_LPF, 3);
                dataShift(u->r_r, 3);


                ///calculate psid rd drd
                psid = atan2(dy,dx);
				if( (dx)!=0.0 || (dy)!=0.0 ){
					
					rd = (ddy*dx-dy*ddx)/(dx*dx+dy*dy);
					drd = (((dddy)*(dx)-(dy)*(dddx))*((dx)*(dx)+(dy)*(dy)) - ((ddy)*(dx)-(dy)*(ddx))*(2*(dx)*(ddx)+2*(dy)*(ddy))) / mySq((dx)*(dx)+(dy)*(dy));
				}
				else{
					rd = 0.0;
					drd = 0.0;
				}

                ///calculate dy_r
                dy_real = -cg.dx[0]*sin(psid)+cg.dy[0]*cos(psid)-rd*(cg.x*cos(psid)+cg.y*sin(psid));
                dyd_r = -dx*sin(psid)+dy*cos(psid)-rd*(x*cos(psid)+y*sin(psid));
				u->dy_r[0] =dy_real - dyd_r; 
                ///calculate ddy_r
                ddy_real = -cg.ddx[0]*sin(psid)+cg.ddy[0]*cos(psid)-2*rd*(cg.dx[0]*cos(psid)+cg.dy[0]*sin(psid))-rd*rd*(-cg.x*sin(psid)+cg.y*cos(psid))-drd*(cg.x*cos(psid)+cg.y*sin(psid));
                ddyd_r = -ddx*sin(psid)+ddy*cos(psid)-2*rd*(dx*cos(psid)+dy*sin(psid))-rd*rd*(-x*sin(psid)+y*cos(psid))-drd*(x*cos(psid)+y*sin(psid));
                u->ddy_r[0] = ddy_real-ddyd_r;

                //judge direction of road error
            	COX = cox*cos(atan2(dy,dx))+coy*sin(atan2(dy,dx));
            	COY = -cox*sin(atan2(dy,dx))+coy*cos(atan2(dy,dx));
            	j_dir = COX*(P-x)+COY*(Q-y);

                y_r = sqrt((x-P)*(x-P)+(y-Q)*(y-Q));
                if(j_dir>=0){
                	y_r = y_r;
                }
                else{
                	y_r = -y_r;
                }
                ////////////////////
                u->ccount = ccount;
                u->c_x = x;
                u->c_y = y;
                u->y_r[0] = y_r;
                u->psi_r[0] = cg.psi - psid;
                u->r_r[0]=cg.rLPF[0]-rd;

                //derivative(u->y_r,u->dy_r);
                //derivative(u->dy_r,u->ddy_r);
                //derivative(u->psi_r,u->r_r);
                LPfilter(u->y_r, u->y_r_LPF); 
                LPfilter(u->dy_r, u->dy_r_LPF);
                //LPfilter(u->ddy_r, u->ddy_r_LPF);
                u->ddy_r_LPF[0] = u->ddy_r[0];
                LPfilter(u->psi_r, u->psi_r_LPF);
                //printf("sec = %f,t_new=%f,y_r=%f,dy_r=%f,psi_r=%f, r_r=%f\n",sec,t_new,u->y_r[0],u->dy_r[0],u->psi_r[0],u->r_r[0]);
                //printf("sec = %f,t_new=%f,t=%f,x=%f,y=%f,P=%f,Q=%f,y_r=%f,dy_r=%f,psi_r=%f, r_r=%f\n",sec,t_new,t,x,y,P,Q,u->y_r[0],u->dy_r_LPF[0],u->psi_r[0],u->r_r_LPF[0]);
                
                //printf("t_new=%f, count = %d,x=%f,y=%f,error=%f,y_r=%f,dy_r=%f,psi_r=%f, r_r=%f\n",t_new,ccount,x,y,error,u->y_r[0],u->dy_r[0],u->psi_r[0],u->r_r[0]);
                //return 1;
                break;
            }


            t = t_new;
            ccount = ccount + 1;

        }
        return 1;

    }
    else if (sec<t4){

    	printf("slow down\n");
        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);
        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_LPF, 3);
        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);



        dx = vth - a0*(t-t3);
        x = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 +v*(t2-t1)+(v+vth)*(t3-t2)/2 + (vth+dx)*(t-t3)/2;
        ddx = -a0;

        H = tanh(b*(x-0.5*L-x_move));
        dH = (1-H*H)*b*dx;

        y = a*H+0.5*W;
        dy = a*dH;

        y_r = Q-y;
        u->ccount = ccount;
        u->c_x = x;
        u->c_y = y;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-0;
        u->r_r[0] = cg.rLPF[0] - 0;

        ///calculate dy_r
        dy_real = cg.dy[0];
        dyd_r = dy;
        u->dy_r[0] = dy_real - dyd_r;   


        //derivative(u->y_r,u->dy_r);
        //derivative(u->dy_r,u->ddy_r);
        //derivative(u->psi_r,u->r_r);
        //printf("t4=%f  t=%f, count = %d,x=%f,y=%f,error=%f,y_r=%f,dy_r=%f,psi_r=%f, r_r=%f\n",t4,t,ccount,x,y,error,u->y_r[0],u->dy_r[0],u->psi_r[0],u->r_r[0]);
        LPfilter(u->y_r, u->y_r_LPF);
        LPfilter(u->dy_r, u->dy_r_LPF);
        LPfilter(u->psi_r, u->psi_r_LPF);
        //LPfilter(u->r_r, u->r_r_LPF);

        return 1;
    }
    else{

        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);
        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_LPF, 3);

        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);




        x = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 +v*(t2-t1)+(v+vth)*(t3-t2)/2 + vth*(t4-t3)/2;
        dx = 0;
        H = tanh(b*(x-0.5*L-x_move));
        dH = (1-H*H)*b*dx;

        y = a*H+0.5*W;
        dy = a*dH;

        y_r = Q-y;
        u->ccount = ccount;
        u->c_x = x;
        u->c_y = y;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-0;
        u->r_r[0] = cg.rLPF[0] - 0;

        ///calculate dy_r
        dy_real = cg.dy[0];
        dyd_r = dy;
        u->dy_r[0] = dy_real - dyd_r;  
        //derivative(u->y_r,u->dy_r);
        //derivative(u->dy_r,u->ddy_r);
        //derivative(u->psi_r,u->r_r);
        LPfilter(u->y_r, u->y_r_LPF);
        LPfilter(u->dy_r, u->dy_r_LPF);
        LPfilter(u->psi_r, u->psi_r_LPF);
        //LPfilter(u->r_r, u->r_r_LPF);


        printf("over t4\n");

        return 1;
    }



}

// ================================================================
//                      calculate_road_error_genDesiredTrajectory7
// ================================================================

int calculate_road_error_genDesiredTrajectory7(CG cg, UPPERLAYER_DATA *u,double sec,double L,double v){

    // static double L;
    static double x,dx,ddx,dddx;
    static double P,Q;
    static double y_r;

    static double vth; 
    static double t0;
    static double t01;
    static double t;

	static double a0,a1,t1,t2 , t3, t4;
	
	double rd,drd,ddyd_r,ddy_real;
	double dyd_r,dy_real;
	double L1,L2;

	double psid;

    ////
	vth = 0.7;
	t0 = 0.5;

   

/*
    L=5;
    W= 0.7;
    b= 1;

*/
	a0 = vth/t0;
	a1 = (v-vth)/(t1-t01);

	t01 = t0+1;
    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+1;
	t4 = t3+t0;


    static double t_new;
    static double f_ans;
    static int ccount = 1;
    P = cg.x;
    Q = cg.y;

    if(sec<t4){


        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);
        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_LPF, 3);
        //dataShift(u->ddy_r, 2);
        dataShift(u->ddy_r, 3);
        
        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);



        y_r = Q-0;
        u->ccount = ccount;
        u->c_x = P;
        u->c_y = 0;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-0;
        u->r_r[0] = cg.rLPF[0] - 0;

        ///calculate dy_r
        dy_real = cg.dy[0];
        dyd_r = 0;
        u->dy_r[0] = dy_real - dyd_r;
        ///calculate ddy_r

        ddy_real = cg.ddy[0];
        ddyd_r = 0;
        u->ddy_r[0] = ddy_real-ddyd_r;

 		u->ddy_r_LPF[0] = u->ddy_r[0];
        LPfilter(u->y_r, u->y_r_LPF);   
        LPfilter(u->dy_r, u->dy_r_LPF);
        LPfilter(u->psi_r, u->psi_r_LPF);
        //LPfilter(u->r_r, u->r_r_LPF);


        return 1;
    }
    else{

        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);
        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_LPF, 3);
        //dataShift(u->ddy_r, 2);
        dataShift(u->ddy_r, 3);
        
        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);



        y_r = Q-0;
        u->ccount = ccount;
        u->c_x = P;
        u->c_y = 0;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-0;
        u->r_r[0] = cg.rLPF[0] - 0;

        ///calculate dy_r
        dy_real = cg.dy[0];
        dyd_r = 0;
        u->dy_r[0] = dy_real - dyd_r;
        ///calculate ddy_r

        ddy_real = cg.ddy[0];
        ddyd_r = 0;
        u->ddy_r[0] = ddy_real-ddyd_r;
        //derivative(u->y_r,u->dy_r);
        //derivative(u->dy_r,u->ddy_r);
        //derivative(u->psi_r,u->r_r);   
        LPfilter(u->y_r, u->y_r_LPF);   
        LPfilter(u->dy_r, u->dy_r_LPF);
        LPfilter(u->psi_r, u->psi_r_LPF);
        //LPfilter(u->r_r, u->r_r_LPF);


        

        printf("over t4\n");

        return 1;
    }



}


// ================================================================
//                      calculate_road_error_genDesiredTrajectory8
// ================================================================

int calculate_road_error_genDesiredTrajectory8(CG cg, UPPERLAYER_DATA *u,double sec,double L,double v,double R){

    
    static double x,dx,ddx,dddx;
    static double y,dy,ddy,dddy;
    static double f,f_dot,error;
    static double P,Q;
    static double y_r;

    static double vth; 
    static double t0;
    static double t01;
    static double t;

	static double a0,a1,t1,t2 , t3, t4,t5;
	static double x_move;
	double T,w;
	double rd,drd,ddyd_road,ddy_real;
	double dyd_road,dy_real;

	double L1,L2;
	double cox,coy;
	double COX,COY;
	double j_dir;
	double psid;

	//judge direction
	cox = 0;
    coy = 1;

  	w = v/R;
  	T = 2*PI/w;
 	

	vth = 0.7;

	t0 = 0.5;
	t01 = t0+1;
    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+0.25*T;
	t4 = t3+1;
	t5 = t4+t0;
    ////


	a0 = vth/t0;
	a1 = (v-vth)/(t1-t01);

  	x_move = 0.5*a0*t0*t0+vth*(t01-t0)+(vth+v)*(t1-t01)/2 + v*(t2-t1);




    static double t_new;
    static double f_ans;
    static int ccount = 1;
    P = cg.x;
    Q = cg.y;


    if(sec<t2){


        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);

        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_bridge, 3);
        dataShift(u->dy_r_LPF, 3);
        //dataShift(u->ddy_r, 2);
        dataShift(u->ddy_r, 3);
        dataShift(u->ddy_r_bridge, 3);
        dataShift(u->ddy_r_LPF, 3);

        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);
        dataShift(u->r_r_bridge, 2);



        y_r = Q-0;
        u->ccount = ccount;
        u->c_x = P;
        u->c_y = 0;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-0;
        u->r_r_bridge[0]= cg.rLPF[0] - 0;
        LPfilter_1_order(u->r_r_bridge,u->r_r);
        
        ///calculate dy_r
        dy_real = cg.dy[0];
        dyd_road = 0;
        u->dy_r[0] = dy_real - dyd_road;
        ///calculate ddy_r

        ddy_real = cg.ddy[0];
        ddyd_road = 0;
        u->ddy_r[0] = ddy_real-ddyd_road;

 		//u->ddy_r_LPF[0] = u->ddy_r[0];
        LPfilter(u->y_r, u->y_r_LPF);   
        LPfilter(u->dy_r, u->dy_r_bridge);
        LPfilter_1_order(u->dy_r_bridge,u->dy_r_LPF);

        LPfilter(u->ddy_r, u->ddy_r_bridge);
        LPfilter_1_order(u->ddy_r_bridge,u->ddy_r_LPF);
        LPfilter(u->psi_r, u->psi_r_LPF);
        //LPfilter(u->r_r, u->r_r_LPF);



        return 1;
    }
    else if(sec<t3){

        printf("calculate road error\n");
        t=sec;

        while(1){
			x = x_move + R*cos(w*(t-t2+0.75*T));
			dx = -R*w*sin(w*(t-t2+0.75*T)) ;
			ddx = -R*w*w*cos(w*(t-t2+0.75*T));
			dddx = R*w*w*w*sin(w*(t-t2+0.75*T)) ;

			y = R+R*sin(w*(t-t2+0.75*T));
			dy = R*w*cos(w*(t-t2+0.75*T)) ;
			ddy = -R*w*w*sin(w*(t-t2+0.75*T));;
			dddy = -R*w*w*w*cos(w*(t-t2+0.75*T)) ;

			// psid = atan2(dy,dx);
			// if( (dx)!=0.0 || (dy)!=0.0 ){
			// 	rd = ((ddy)*(dx) - (dy)*(ddx))/((dx)*(dx) + (dy)*(dy));
			// 	drd = (((dddy)*(dx)-(dy)*(dddx))*((dx)*(dx)+(dy)*(dy)) - ((ddy)*(dx)-(dy)*(ddx))*(2*(dx)*(ddx)+2*(dy)*(ddy))) / mySq((dx)*(dx)+(dy)*(dy));
			// }
			// else{
			// 	rd = 0.0;
			// 	drd = 0.0;
			// }

            
            L1=sqrt((P-x)*(P-x)+(Q-y)*(Q-y));
            L2=sqrt(dx*dx+dy*dy);
            f=((P-x)*dx+(Q-y)*dy)/(L1*L2);
            f_dot =(-dx*dx+(P-x)*ddx - dy*dy+(Q-y)*ddy)/(L1*L2);
            f_ans = f/f_dot;

            t_new = t - f/f_dot;

            error = fabs(t_new - t);
            //error<0.00001 || t==0.5
            if(error<0.00001){


                printf("find  count=%d\n",ccount);
                //printf("count = %d,x=%f,y=%f,error=%f\n",count,x,y,error);
                dataShift(u->y_r, 3);
                dataShift(u->y_r_LPF, 3);

                dataShift(u->dy_r, 3);
                dataShift(u->dy_r_bridge, 3);
                dataShift(u->dy_r_LPF, 3);

                dataShift(u->ddy_r, 3);
                dataShift(u->ddy_r_bridge, 3);
                dataShift(u->ddy_r_LPF, 3);

        		dataShift(u->psi_r, 3);
        		dataShift(u->psi_r_LPF, 3);
                dataShift(u->r_r, 3);
                dataShift(u->r_r_bridge, 2);




            


                ///calculate psid rd drd
                psid = atan2(dy,dx);
				if( (dx)!=0.0 || (dy)!=0.0 ){
					
					rd = (ddy*dx-dy*ddx)/(dx*dx+dy*dy);
					drd = (((dddy)*(dx)-(dy)*(dddx))*((dx)*(dx)+(dy)*(dy)) - ((ddy)*(dx)-(dy)*(ddx))*(2*(dx)*(ddx)+2*(dy)*(ddy))) / mySq((dx)*(dx)+(dy)*(dy));
				}
				else{
					rd = 0.0;
					drd = 0.0;
				}

       
                ///calculate dy_r
                dy_real = -cg.dx[0]*sin(psid)+cg.dy[0]*cos(psid);
                dyd_road = -dx*sin(psid)+dy*cos(psid);
				u->dy_r[0] =dy_real - dyd_road; 
                ///calculate ddy_r
                ddy_real = -cg.ddx[0]*sin(psid)+cg.ddy[0]*cos(psid)-rd*(cg.dx[0]*cos(psid)+cg.dy[0]*sin(psid));
                ddyd_road = -ddx*sin(psid)+ddy*cos(psid)-rd*(dx*cos(psid)+dy*sin(psid));
                u->ddy_r[0] = ddy_real-ddyd_road;

                //judge direction of road error
            	COX = cox*cos(atan2(dy,dx))+coy*sin(atan2(dy,dx));
            	COY = -cox*sin(atan2(dy,dx))+coy*cos(atan2(dy,dx));
            	j_dir = COX*(P-x)+COY*(Q-y);

                y_r = sqrt((x-P)*(x-P)+(y-Q)*(y-Q));
                if(j_dir>=0){
                	y_r = y_r;
                }
                else{
                	y_r = -y_r;
                }
                ////////////////////

                u->ccount = ccount;
                u->c_x = x;
                u->c_y = y;
                u->y_r[0] = y_r;
                u->psi_r[0] = cg.psi - psid;
                
                u->r_r_bridge[0]= cg.rLPF[0] - rd;
        		LPfilter_1_order(u->r_r_bridge,u->r_r);
                //u->ddy_r_LPF[0] = u->ddy_r[0];

                LPfilter(u->y_r, u->y_r_LPF); 
		        LPfilter(u->dy_r, u->dy_r_bridge);
		        LPfilter_1_order(u->dy_r_bridge,u->dy_r_LPF);

		        LPfilter(u->ddy_r, u->ddy_r_bridge);
		        LPfilter_1_order(u->ddy_r_bridge,u->ddy_r_LPF);

                LPfilter(u->psi_r, u->psi_r_LPF);

                break;
            }

            else if(ccount > 8){

                printf("over 8 times\n");


                dataShift(u->y_r, 3);
                dataShift(u->y_r_LPF, 3);

                dataShift(u->dy_r, 3);
                dataShift(u->dy_r_bridge, 3);
                dataShift(u->dy_r_LPF, 3);

                dataShift(u->ddy_r, 3);
                dataShift(u->ddy_r_bridge, 3);
                dataShift(u->ddy_r_LPF, 3);

        		dataShift(u->psi_r, 3);
        		dataShift(u->psi_r_LPF, 3);
                dataShift(u->r_r, 3);
                dataShift(u->r_r_bridge, 2);


                ///calculate psid rd drd
                psid = atan2(dy,dx);
				if( (dx)!=0.0 || (dy)!=0.0 ){
					
					rd = (ddy*dx-dy*ddx)/(dx*dx+dy*dy);
					drd = (((dddy)*(dx)-(dy)*(dddx))*((dx)*(dx)+(dy)*(dy)) - ((ddy)*(dx)-(dy)*(ddx))*(2*(dx)*(ddx)+2*(dy)*(ddy))) / mySq((dx)*(dx)+(dy)*(dy));
				}
				else{
					rd = 0.0;
					drd = 0.0;
				}

                ///calculate dy_r
                dy_real = -cg.dx[0]*sin(psid)+cg.dy[0]*cos(psid);
                dyd_road = -dx*sin(psid)+dy*cos(psid);
				u->dy_r[0] =dy_real - dyd_road; 
                ///calculate ddy_r
                ddy_real = -cg.ddx[0]*sin(psid)+cg.ddy[0]*cos(psid)-rd*(cg.dx[0]*cos(psid)+cg.dy[0]*sin(psid));
                ddyd_road = -ddx*sin(psid)+ddy*cos(psid)-rd*(dx*cos(psid)+dy*sin(psid));
                u->ddy_r[0] = ddy_real-ddyd_road;

                //judge direction of road error
            	COX = cox*cos(atan2(dy,dx))+coy*sin(atan2(dy,dx));
            	COY = -cox*sin(atan2(dy,dx))+coy*cos(atan2(dy,dx));
            	j_dir = COX*(P-x)+COY*(Q-y);

                y_r = sqrt((x-P)*(x-P)+(y-Q)*(y-Q));
                if(j_dir>=0){
                	y_r = y_r;
                }
                else{
                	y_r = -y_r;
                }
                ////////////////////
                u->ccount = ccount;
                u->c_x = x;
                u->c_y = y;
                u->y_r[0] = y_r;
                u->psi_r[0] = cg.psi - psid;
                //u->r_r[0]=cg.rLPF[0]-rd;
                //u->ddy_r_LPF[0] = u->ddy_r[0];

                u->r_r_bridge[0]= cg.rLPF[0] - rd;
        		LPfilter_1_order(u->r_r_bridge,u->r_r);

                LPfilter(u->y_r, u->y_r_LPF); 
		        LPfilter(u->dy_r, u->dy_r_bridge);
		        LPfilter_1_order(u->dy_r_bridge,u->dy_r_LPF);

		        LPfilter(u->ddy_r, u->ddy_r_bridge);
		        LPfilter_1_order(u->ddy_r_bridge,u->ddy_r_LPF);

                LPfilter(u->psi_r, u->psi_r_LPF);

                break;
            }


            t = t_new;
            ccount = ccount + 1;

        }
        return 1;

    }
    else if (sec<t4){

    	printf("slow down\n");
        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);

        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_bridge, 3);
        dataShift(u->dy_r_LPF, 3);

        dataShift(u->ddy_r, 3);
        dataShift(u->ddy_r_bridge, 3);
        dataShift(u->ddy_r_LPF, 3);

        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);
        dataShift(u->r_r_bridge, 2);


		x =x_move + R*cos(w*(t3-t2+0.75*T));
        dx = 0;
		ddx = 0;
		dddx = 0;

		dy = R*w*cos(w*(t3-t2+0.75*T)) -a1*(t-t3);
		y = R+R*sin(w*(t3-t2+0.75*T)) + (v+dy)*(t-t3)/2;
		ddy = -a1;
		dddy = 0;

		psid = PI/2;
		rd = 0.0;
		drd = 0.0;
		


        y_r = P-x;
        u->ccount = ccount;
        u->c_x = x;
        u->c_y = y;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-psid;
        u->r_r_bridge[0] = cg.rLPF[0] - rd;
        LPfilter_1_order(u->r_r_bridge,u->r_r);
 		
        ///calculate dy_r
        dy_real = -cg.dx[0]*sin(psid)+cg.dy[0]*cos(psid);
        dyd_road = -dx*sin(psid)+dy*cos(psid);
		u->dy_r[0] =dy_real - dyd_road; 
        ///calculate ddy_r
        ddy_real = -cg.ddx[0]*sin(psid)+cg.ddy[0]*cos(psid)-rd*(cg.dx[0]*cos(psid)+cg.dy[0]*sin(psid));
        ddyd_road = -ddx*sin(psid)+ddy*cos(psid)-rd*(dx*cos(psid)+dy*sin(psid));
        u->ddy_r[0] = ddy_real-ddyd_road;

        LPfilter(u->y_r, u->y_r_LPF);

        LPfilter(u->dy_r, u->dy_r_bridge);
        LPfilter_1_order(u->dy_r_bridge,u->dy_r_LPF);

        LPfilter(u->ddy_r, u->ddy_r_bridge);
        LPfilter_1_order(u->ddy_r_bridge,u->ddy_r_LPF);

        LPfilter(u->psi_r, u->psi_r_LPF);


        return 1;
    }
    else if(t<t5){

    	printf("slow down\n");
        dataShift(u->y_r, 3);
        dataShift(u->y_r_LPF, 3);

        dataShift(u->dy_r, 3);
        dataShift(u->dy_r_bridge, 3);
        dataShift(u->dy_r_LPF, 3);

        dataShift(u->ddy_r, 3);
        dataShift(u->ddy_r_bridge, 3);
        dataShift(u->ddy_r_LPF, 3);

        dataShift(u->psi_r, 3);
        dataShift(u->psi_r_LPF, 3);
        dataShift(u->r_r, 3);
        dataShift(u->r_r_bridge, 2);




		x =x_move + R*cos(w*(t3-t2+0.75*T));
        dx = 0;
		ddx = 0;
		dddx = 0;


		dy = vth - a0*(t-t4);
		y = R+R*sin(w*(t3-t2+0.75*T)) + (v+vth)*(t4-t3)/2 + (vth+dy)*(t-t4)/2;
		ddy = -a0;
		dddy = 0;

		psid = PI/2;
		rd = 0.0;
		drd = 0.0;
		


        y_r = P-x;
        u->ccount = ccount;
        u->c_x = x;
        u->c_y = y;
        u->y_r[0] = y_r;
        u->psi_r[0] = cg.psi-psid;
        
        u->r_r_bridge[0]= cg.rLPF[0] - rd;
        LPfilter_1_order(u->r_r_bridge,u->r_r);

        ///calculate dy_r
        dy_real = -cg.dx[0]*sin(psid)+cg.dy[0]*cos(psid);
        dyd_road = -dx*sin(psid)+dy*cos(psid);
		u->dy_r[0] =dy_real - dyd_road; 
        ///calculate ddy_r
        ddy_real = -cg.ddx[0]*sin(psid)+cg.ddy[0]*cos(psid)-rd*(cg.dx[0]*cos(psid)+cg.dy[0]*sin(psid));
        ddyd_road = -ddx*sin(psid)+ddy*cos(psid)-rd*(dx*cos(psid)+dy*sin(psid));
        u->ddy_r[0] = ddy_real-ddyd_road;
 


        LPfilter(u->y_r, u->y_r_LPF);
        
        LPfilter(u->dy_r, u->dy_r_bridge);
        LPfilter_1_order(u->dy_r_bridge,u->dy_r_LPF);

        LPfilter(u->ddy_r, u->ddy_r_bridge);
        LPfilter_1_order(u->ddy_r_bridge,u->ddy_r_LPF);
        
        LPfilter(u->psi_r, u->psi_r_LPF);


        return 1;
    }



}
// ================================================================
//                      sliding mode control
// ================================================================
int upperLayerControl(CG *cgPtr, UPPERLAYER_DATA *u,double k_gain[][8],double dk_gain[][8],double Pe[][6],double b[],double sec,double L, double v)
{
    static DESIRED_TRAJECTORY *d = &(u->desiredTrajectory);
    dataShift(cgPtr->x_r, 2);
    dataShift(cgPtr->dx_r, 3);
    dataShift(cgPtr->dx_r_LPF, 3);

    dataShift(u->xd_r, 2);
    dataShift(u->dxd_r, 2);
    dataShift(u->integral_delta_x1, 2);
    dataShift(u->integral_delta_x2, 2);
    dataShift(u->law,5);
    dataShift(u->law_dot,5);

   
    double D_1,D_2;
    double t0,t01,t1,t2,t3;

    double eta;
    double eta_last;
    double d23;

    t0 = 0.5;
    t01 = t0 + 1;
    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+1;


    d->curvature=sqrt(pow(fabs((d->ddyd/(1+(pow(d->dyd,2))))),3));

 	static int i;
    i = 0;
    if(sec>=0.5){
    	i=(int)((sec-t0)*100);
    }   
	

	u->dxd_r[0] = (d->dxd)*cos(d->psid) + (d->dyd)*sin(d->psid);
	///dxd_r integral = xd_r
	u->xd_r[0] =u->xd_r[1] + u->dxd_r[0]*0.01;

	u->dyd_r = -(d->dxd)*sin(d->psid) + (d->dyd)*cos(d->psid);
	u->ddxd_r = (d->ddxd)*cos(d->psid) + (d->ddyd)*sin(d->psid)+(d->rd)*u->dyd_r;

	cgPtr->dx_r[0] = (cgPtr->dx[0])*cos(d->psid) + (cgPtr->dy[0])*sin(d->psid);
	LPfilter(cgPtr->dx_r, cgPtr->dx_r_LPF);
	//cgPtr->dx_r_LPF[0] = cgPtr->VXLPF[0];
	///dx_r_LPF integral is x_r
	cgPtr->x_r[0] = cgPtr->x_r[1] + cgPtr->dx_r_LPF[0]*0.01;

    //u->FX = -MASS*(-u->ddxd_r+D_1+ks1*saturation(u->SV1, epsilonSV1_U, epsilonSV1_L)+lemda1*(u->SV1)+u->dy_r[0]*cg.dx_r*d->curvature);
    if(sec<t0){
        u->FX = -MASS*(-u->ddxd_r)+MASS*1.5; //+MASS*1.5
        u->M = -IZ*(1*u->r_r[0]+1*u->psi_r_LPF[0]);
        //u->M = -IZ*(1*u->r_r[0]+1*u->psi_r[0]);
    }
    else if(sec<t3){
	///// law design

    	eta = (b[i]*(u->y_r_LPF[0]*Pe[i][0]+u->dy_r_LPF[0]*Pe[i][1]+u->psi_r_LPF[0]*Pe[i][2])+(u->y_r_LPF[0]*Pe[i][3]+u->dy_r_LPF[0]*Pe[i][4]+u->psi_r_LPF[0]*Pe[i][5]))*b[i]/(b[i]*b[i]+1);
    	if(fabs(eta)>=epsilonPlus){
    		eta_last = eta;
    	}
    	else if(eta >= 0){
    		eta_last = epsilonPlus;
    	}
    	else if(eta < 0){
    		eta_last = - epsilonPlus;
    	}
    	u->eta_last_matric = eta_last;

		u->law[0] = d2_bar*fabs(u->y_r_LPF[0]*Pe[i][0]+u->dy_r_LPF[0]*Pe[i][1]+u->psi_r_LPF[0]*Pe[i][2])*b[i]/((b[i]*b[i]+1)*eta_last);
		differentiator(u->law, u->law_dot);
		// if(i==0){
		// 	u->law_dot[i] = 0;
		// }
		// else{
		// 	u->law_dot[i] = (u->law[i]-u->law[i-1])/0.01;
		// }
		
		// u->law_dot = d23*epsilonPlus/((k_gain[i][6]+epsilonPlus)*(k_gain[i][6]+epsilonPlus));

        u->delta_x1 = (cgPtr->dx_r_LPF[0] - u->dxd_r[0])+k_gain[i][0]*(cgPtr->x_r[0] - u->xd_r[0])+k_gain[i][1]*(u->y_r_LPF[0])+k_gain[i][2]*(u->dy_r_LPF[0])+k_gain[i][3]*(u->psi_r_LPF[0]);
        u->integral_delta_x1[0] = u->integral_delta_x1[1] + sqrt3(u->delta_x1)*0.01;
        u->SV1= u->delta_x1 + C1*u->integral_delta_x1[0];

        u->delta_x2 = u->r_r[0]+k_gain[i][4]*(cgPtr->x_r[0] - u->xd_r[0])+k_gain[i][5]*(u->y_r_LPF[0])+k_gain[i][6]*(u->dy_r_LPF[0])+k_gain[i][7]*(u->psi_r_LPF[0])+u->law[0];
        u->integral_delta_x2[0] = u->integral_delta_x2[1] + sqrt3(u->delta_x2)*0.01;
        u->SV2=u->delta_x2 + C2*u->integral_delta_x2[0];

        D_1 = dk_gain[i][0]*(cgPtr->x_r[0] - u->xd_r[0])+dk_gain[i][1]*(u->y_r_LPF[0])+dk_gain[i][2]*(u->dy_r_LPF[0])+dk_gain[i][3]*(u->psi_r_LPF[0])+k_gain[i][0]*(cgPtr->dx_r_LPF[0]- u->dxd_r[0])+k_gain[i][1]*(u->dy_r_LPF[0])+k_gain[i][2]*(u->ddy_r_LPF[0])+k_gain[i][3]*(u->r_r[0])+C1*sqrt3(u->SV1);
        D_2 = dk_gain[i][4]*(cgPtr->x_r[0] - u->xd_r[0])+dk_gain[i][5]*(u->y_r_LPF[0])+dk_gain[i][6]*(u->dy_r_LPF[0])+dk_gain[i][7]*(u->psi_r_LPF[0])+k_gain[i][4]*(cgPtr->dx_r_LPF[0]- u->dxd_r[0])+k_gain[i][5]*(u->dy_r_LPF[0])+k_gain[i][6]*(u->ddy_r_LPF[0])+k_gain[i][7]*(u->r_r[0])+C2*sqrt3(u->SV2);

        if(abs(u->SV1) > 0.4) {
            u->lemda1 = lemda1_tck;
        }else {
            u->lemda1 = lemda1_amcl;
        }
        // u->lemda1 = lemda1_f;
        
        if(sec > 3.0){
            // u->lemda1 = lemda1_tck;
            u->ks1 = ks1_tck;
            // u->ks1 = ks1_amcl;
            // u->ks1 = ks1_s;
        }else{
            // u->lemda1 = lemda1_tck;
            u->ks1 = ks1_tck;
            // u->ks1 = ks1_amcl;
            // u->ks1 = ks1_s;
        }

        if(sec > 4.0){
            // u->lemda2 = lemda2_s;
            u->lemda2 = lemda2_f;
        }else{
            // u->lemda2 = lemda2_s;
            u->lemda2 = lemda2_f;
        }


        u->FX = -MASS*(-u->ddxd_r+D_1+u->ks1*saturation(u->SV1, epsilonSV1_U, epsilonSV1_L)+u->lemda1*(u->SV1)+u->dy_r_LPF[0]*cgPtr->dx_r_LPF[0]*d->curvature);
        
        u->M = -IZ*((-(2*(LF*Cyf-LR*Cyr)/IZ)*u->dy_r_LPF[0]/(cgPtr->VXLPF[0]))+(2*(LF*Cyf-LR*Cyr)*u->psi_r_LPF[0]/IZ)-((2*(LF*LF*Cyf+LR*LR*Cyr)/IZ)*u->r_r[0]/(cgPtr->VXLPF[0]))+D_2+ks2*saturation(u->SV2, epsilonSV2_U, epsilonSV2_L)+u->lemda2*u->SV2+u->law_dot[0]);

    }
    else{ 
        u->FX = -MASS*(-u->ddxd_r)-MASS*1.5;
        u->M = -IZ*(1*u->r_r[0]+1*u->psi_r_LPF[0]);
    }

    printf("\n");
    //printf("upperlayer\n");
    printf("sec=%f\n",sec);
    printf("\n");

    
    printf("FX\n");
    //printf("ddxd_r = %f,ddy_r=%f, D_1=%f,ks1*sat=%f,lemda1*sv1=%f, dy_r*curvature=%f\n ", -u->ddxd_r,u->ddy_r[0],D_1,ks1*saturation(u->SV1, epsilonSV1_U, epsilonSV1_L),lemda1*(u->SV1),u->dy_r[0]*cg.dx_r*d->curvature);

    printf("1=%f,2=%f,3=%f ,4=%f\n",dk_gain[i][0]*(cgPtr->x_r[0] - u->xd_r[0]),dk_gain[i][1]*(u->y_r_LPF[0]),dk_gain[i][2]*(u->dy_r_LPF[0]),dk_gain[i][3]*(u->psi_r_LPF[0]));
    printf("5=%f ,6=%f,7=%f,8=%f\n",k_gain[i][0]*(cgPtr->dx_r_LPF[0] - u->dxd_r[0]),k_gain[i][1]*(u->dy_r_LPF[0]),k_gain[i][2]*(u->ddy_r_LPF[0]),k_gain[i][3]*(u->r_r[0]));
    //printf("k_gain[i][3]=%f\n",k_gain[i][3]);
/*	
	printf("\n");    
	
    printf("M = %f \n",u->M);
    printf("1 = %f, 2 = %f, 3=%f, D_2=%f,ks2*sat=%f, lemda2*sv2=%f \n",-(2*(LF*Cyf-LR*Cyr)/IZ)*u->dy_r_LPF[0]/(cg.VXLPF[0]),(2*(LF*Cyf-LR*Cyr)*u->psi_r_LPF[0]/IZ),-(2*(LF*LF*Cyf+LR*LR*Cyr)/IZ)*u->r_r[0]/(cg.VXLPF[0]),D_2,ks2*saturation(u->SV2, epsilonSV2_U, epsilonSV2_L),lemda2*u->SV2);

    printf("D2 1 = %f, 2 = %f, 3=%f ,4=%f", dk_gain[i][4]*(cg.x_r[0] - u->xd_r[0]),dk_gain[i][5]*(u->y_r_LPF[0]),dk_gain[i][6]*(u->dy_r_LPF[0]),dk_gain[i][7]*(u->psi_r_LPF[0]));
    printf("D2 5 = %f, 6 = %f, 7=%f ,8=%f", k_gain[i][4]*(cg.dx_r_LPF[0] - u->dxd_r[0]),k_gain[i][5]*(u->dy_r_LPF[0]),k_gain[i][6]*(u->ddy_r_LPF[0]),k_gain[i][7]*(u->r_r[0]) );
//    printf("(cg.dx_r)=%f\n",cg.dx_r);
    //printf("(cg.dx_r+u->dy_r[0]*u->psi_r[0])=%f\n",(cg.dx_r+u->dy_r_LPF[0]*u->psi_r_LPF[0]));
*/	
    printf("(cg.dx_r)=%f\n",cgPtr->dx_r_LPF[0]);
	u->F_D1=D_1;
	u->F_ks1sat=u->ks1*saturation(u->SV1, epsilonSV1_U, epsilonSV1_L);
    u->F_lemda1SV1=u->lemda1*(u->SV1);
	u->F_dy_r_dx_r_curvature=u->dy_r_LPF[0]*cgPtr->dx_r_LPF[0]*d->curvature;

	u->M_1=-(2*(LF*Cyf-LR*Cyr)/IZ)*u->dy_r_LPF[0]/(cgPtr->VXLPF[0]);
	u->M_2=(2*(LF*Cyf-LR*Cyr)*u->psi_r[0]/IZ);
	u->M_3=-(2*(LF*LF*Cyf+LR*LR*Cyr)/IZ)*u->r_r[0]/(cgPtr->VXLPF[0]);
	u->M_D2=D_2;
	u->M_ks2sat=ks2*saturation(u->SV2, epsilonSV2_U, epsilonSV2_L);
	u->M_lemda2SV2=u->lemda2*u->SV2;

	// robot no rotate at original
    // u->FX = 0; 
    // u->M = 0;	
    
 	// tracker calibration 
   // if(sec<8){
   //      u->FX = 130; //+MASS*1.5
   //      u->M = 0;
   //      //u->M = -IZ*(1*u->r_r[0]+1*u->psi_r[0]);
   //  }
   //  else if(cgPtr->VXLPF[0]>=1.2){
   //      u->FX = 0; //+MASS*1.5
   //      u->M = 0;    	
   //  }
   //  else{
   //   	u->FX = -100;
   // 		 u->M = 0;   	
   //  }   
    return 1;

}


// ================================================================
//                      sliding mode control trajectory: genDesiredTrajectory8
// ================================================================
int upperLayerControl_genDesiredTrajectory8(CG *cgPtr, UPPERLAYER_DATA *u,double k_gain[][8],double dk_gain[][8],double k_gain_plus[][8],double dk_gain_plus[][8],double Pe[][6],double b[],double sec,double L, double v,double R)
{
    static DESIRED_TRAJECTORY *d = &(u->desiredTrajectory);
    dataShift(cgPtr->x_r, 2);
    dataShift(cgPtr->dx_r, 3);
    dataShift(cgPtr->dx_r_LPF, 3);

    dataShift(u->xd_r, 2);
    dataShift(u->dxd_r, 2);    
    dataShift(u->integral_delta_x1, 2);
    dataShift(u->integral_delta_x2, 2);    

    dataShift(u->law,5);
    dataShift(u->law_dot,5);
    double D_1,D_2;
    double t0,t01,t1,t2,t3,t4,t5;
    double w,T;
    
    double eta;
    double eta_last;
    double d23;
    // double C2;
    // double lemda2;

    // lemda2 = 25.0;
    // if((u->SV2 >= epsilonSV2_U) || (u->SV2 <= -epsilonSV2_L)){

    // 	lemda2= 40.0; //15.0

    // }

  	w = v/R;
  	T = 2*PI/w;

	t0 = 0.5;
	t01 = t0 + 1;
    t1 = t01+1;
	t2 = t01+L/v;
	t3 = t2+0.25*T;
	t4 = t3+1;
	t5 = t4+t0;


    
    d->curvature=sqrt(pow(fabs((d->ddyd/(1+(pow(d->dyd,2))))),3));

	static int i;
    i = 0;
    if(sec>=0.5){
    	i=(int)((sec-0.5)*100);
    }
    

	u->dxd_r[0] = (d->dxd)*cos(d->psid) + (d->dyd)*sin(d->psid);
	///dxd_r integral = xd_r
	u->xd_r[0] =u->xd_r[1] + u->dxd_r[0]*0.01;

	u->dyd_r = -(d->dxd)*sin(d->psid) + (d->dyd)*cos(d->psid);
	u->ddxd_r = (d->ddxd)*cos(d->psid) + (d->ddyd)*sin(d->psid)+(d->rd)*u->dyd_r;

	cgPtr->dx_r[0] = (cgPtr->dx[0])*cos(d->psid) + (cgPtr->dy[0])*sin(d->psid);
	LPfilter(cgPtr->dx_r, cgPtr->dx_r_LPF);
	///dx_r_LPF integral = x_r
	cgPtr->x_r[0] =cgPtr->x_r[1] + cgPtr->dx_r_LPF[0]*0.01;


/////k gain plus design/////

///////////// 


	// if((cgPtr->x_r[0] - u->xd_r[0])>epsilonD21_U || ((cgPtr->x_r[0] - u->xd_r[0])<-epsilonD21_L)){
	// 	k_gain[i][0] = k_gain[i][0]+d21/(fabs(cgPtr->x_r[0] - u->xd_r[0]));
	// 	// k_gain[i][4] = k_gain[i][4]+d21/(fabs(cgPtr->x_r[0] - u->xd_r[0]));
	// }
	// if((u->y_r_LPF[0])>epsilonD22_U || ((u->y_r_LPF[0])<-epsilonD22_L)){
	// 	// k_gain[i][1] = k_gain[i][1]+d22/(fabs(u->y_r_LPF[0]));
	// 	k_gain[i][5] = k_gain[i][5]+d22/(fabs(u->y_r_LPF[0]));
	// }
	// if((u->dy_r_LPF[0])>epsilonD23_U || ((u->dy_r_LPF[0])<-epsilonD23_L)){
	// 	// k_gain[i][2] = k_gain[i][2]+d23/(fabs(u->dy_r_LPF[0]));
	// 	k_gain[i][6] = k_gain[i][6]+d23/(fabs(u->dy_r_LPF[0]));
	// }
	// if((u->psi_r_LPF[0])>epsilonD24_U || ((u->psi_r_LPF[0])<-epsilonD24_L)){
	// 	// k_gain[i][3] = k_gain[i][3]+d24/(fabs(u->psi_r_LPF[0]));
	// 	k_gain[i][7] = k_gain[i][7]+d24/(fabs(u->psi_r_LPF[0]));
	// }		


	// if(i>0){
	// 	dk_gain[i][0] = (k_gain[i][0] - k_gain[i-1][0])/0.01;
	// 	dk_gain[i][1] = (k_gain[i][1] - k_gain[i-1][1])/0.01;
	// 	dk_gain[i][2] = (k_gain[i][2] - k_gain[i-1][2])/0.01;
	// 	dk_gain[i][3] = (k_gain[i][3] - k_gain[i-1][3])/0.01;
	// 	dk_gain[i][4] = (k_gain[i][4] - k_gain[i-1][4])/0.01;
	// 	dk_gain[i][5] = (k_gain[i][5] - k_gain[i-1][5])/0.01;
	// 	dk_gain[i][6] = (k_gain[i][6] - k_gain[i-1][6])/0.01;
	// 	dk_gain[i][7] = (k_gain[i][7] - k_gain[i-1][7])/0.01;

	// }
	
/////////////


    //u->FX = -MASS*(-u->ddxd_r+D_1+ks1*saturation(u->SV1, epsilonSV1_U, epsilonSV1_L)+lemda1*(u->SV1)+u->dy_r[0]*cg.dx_r*d->curvature);
    if(sec<t0){
        u->FX = -MASS*(-u->ddxd_r)+MASS*1.5; //+MASS*1.5
        u->M = -IZ*(1*u->r_r[0]+1*u->psi_r_LPF[0]);
        //u->M = -IZ*(1*u->r_r[0]+1*u->psi_r[0]);
    }
    else if(sec<t4){
	///// law design

    	eta = (b[i]*(u->y_r_LPF[0]*Pe[i][0]+u->dy_r_LPF[0]*Pe[i][1]+u->psi_r_LPF[0]*Pe[i][2])+(u->y_r_LPF[0]*Pe[i][3]+u->dy_r_LPF[0]*Pe[i][4]+u->psi_r_LPF[0]*Pe[i][5]))*b[i]/(b[i]*b[i]+1);
    	if(fabs(eta)>=epsilonPlus){
    		eta_last = eta;
    	}
    	else if(eta >= 0){
    		eta_last = epsilonPlus;
    	}
    	else if(eta < 0){
    		eta_last = - epsilonPlus;
    	}
    	u->eta_last_matric = eta_last;

		u->law[0] = d2_bar*fabs(u->y_r_LPF[0]*Pe[i][0]+u->dy_r_LPF[0]*Pe[i][1]+u->psi_r_LPF[0]*Pe[i][2])*b[i]/((b[i]*b[i]+1)*eta_last);
		differentiator(u->law, u->law_dot);
		// if(i==0){
		// 	u->law_dot[i] = 0;
		// }
		// else{
		// 	u->law_dot[i] = (u->law[i]-u->law[i-1])/0.01;
		// }
		
		// u->law_dot = d23*epsilonPlus/((k_gain[i][6]+epsilonPlus)*(k_gain[i][6]+epsilonPlus));

        u->delta_x1 = (cgPtr->dx_r_LPF[0] - u->dxd_r[0])+k_gain[i][0]*(cgPtr->x_r[0] - u->xd_r[0])+k_gain[i][1]*(u->y_r_LPF[0])+k_gain[i][2]*(u->dy_r_LPF[0])+k_gain[i][3]*(u->psi_r_LPF[0]);
        u->integral_delta_x1[0] = u->integral_delta_x1[1] + sqrt3(u->delta_x1)*0.01;
        u->SV1= u->delta_x1 + C1*u->integral_delta_x1[0];

        u->delta_x2 = u->r_r[0]+k_gain[i][4]*(cgPtr->x_r[0] - u->xd_r[0])+k_gain[i][5]*(u->y_r_LPF[0])+k_gain[i][6]*(u->dy_r_LPF[0])+k_gain[i][7]*(u->psi_r_LPF[0])+u->law[0];
        u->integral_delta_x2[0] = u->integral_delta_x2[1] + sqrt3(u->delta_x2)*0.01;
        u->SV2=u->delta_x2 + C2*u->integral_delta_x2[0];

        D_1 = dk_gain[i][0]*(cgPtr->x_r[0] - u->xd_r[0])+dk_gain[i][1]*(u->y_r_LPF[0])+dk_gain[i][2]*(u->dy_r_LPF[0])+dk_gain[i][3]*(u->psi_r_LPF[0])+k_gain[i][0]*(cgPtr->dx_r_LPF[0]- u->dxd_r[0])+k_gain[i][1]*(u->dy_r_LPF[0])+k_gain[i][2]*(u->ddy_r_LPF[0])+k_gain[i][3]*(u->r_r[0])+C1*sqrt3(u->SV1);
        D_2 = dk_gain[i][4]*(cgPtr->x_r[0] - u->xd_r[0])+dk_gain[i][5]*(u->y_r_LPF[0])+dk_gain[i][6]*(u->dy_r_LPF[0])+dk_gain[i][7]*(u->psi_r_LPF[0])+k_gain[i][4]*(cgPtr->dx_r_LPF[0]- u->dxd_r[0])+k_gain[i][5]*(u->dy_r_LPF[0])+k_gain[i][6]*(u->ddy_r_LPF[0])+k_gain[i][7]*(u->r_r[0])+C2*sqrt3(u->SV2);

        if(abs(u->SV1) > lemda1_f) {
            u->lemda1 = lemda1_s;
        }else {
            u->lemda1 = 0.4;
        }

        if(sec > 4.0){
            u->lemda2 = lemda2_f;
        }else{
            u->lemda2 = lemda2_s;
        }

        u->FX = -MASS*(-u->ddxd_r+D_1+u->ks1*saturation(u->SV1, epsilonSV1_U, epsilonSV1_L)+u->lemda1*(u->SV1)+u->dy_r_LPF[0]*cgPtr->dx_r_LPF[0]*d->curvature);
        u->M = -IZ*((-(2*(LF*Cyf-LR*Cyr)/IZ)*u->dy_r_LPF[0]/(cgPtr->VXLPF[0]))+(2*(LF*Cyf-LR*Cyr)*u->psi_r_LPF[0]/IZ)-((2*(LF*LF*Cyf+LR*LR*Cyr)/IZ)*u->r_r[0]/(cgPtr->VXLPF[0]))+D_2+ks2*saturation(u->SV2, epsilonSV2_U, epsilonSV2_L)+u->lemda2*u->SV2+u->law_dot[0]);

    }
    else{ 
        u->FX = -MASS*(-u->ddxd_r)-MASS*1.5;
        u->M = -IZ*(1*u->r_r[0]+1*u->psi_r_LPF[0]);
    }

    if(u->M >= 200){
    	u->M = 200;

    }
    else if(u->M <= -200){

    	u->M = -200;
    }



    printf("\n");
    //printf("upperlayer\n");
    printf("sec=%f\n",sec);
    printf("\n");


    printf("M = %f \n",u->M);
    printf("1 = %f, 2 = %f, 3=%f, D_2=%f,ks2*sat=%f, lemda2*sv2=%f \n",-(2*(LF*Cyf-LR*Cyr)/IZ)*u->dy_r_LPF[0]/(cgPtr->VXLPF[0]),(2*(LF*Cyf-LR*Cyr)*u->psi_r_LPF[0]/IZ),-(2*(LF*LF*Cyf+LR*LR*Cyr)/IZ)*u->r_r[0]/(cgPtr->VXLPF[0]),D_2,ks2*saturation(u->SV2, epsilonSV2_U, epsilonSV2_L),u->lemda2*u->SV2);

    printf("D2 1 = %f, 2 = %f, 3=%f ,4=%f", dk_gain[i][4]*(cgPtr->x_r[0] - u->xd_r[0]),dk_gain[i][5]*(u->y_r_LPF[0]),dk_gain[i][6]*(u->dy_r_LPF[0]),dk_gain[i][7]*(u->psi_r_LPF[0]));
    printf("D2 5 = %f, 6 = %f, 7=%f ,8=%f", k_gain[i][4]*(cgPtr->dx_r_LPF[0]- u->dxd_r[0]),k_gain[i][5]*(u->dy_r_LPF[0]),k_gain[i][6]*(u->ddy_r_LPF[0]),k_gain[i][7]*(u->r_r[0]) );
//    printf("(cg.dx_r)=%f\n",cg.dx_r);
    //printf("(cg.dx_r+u->dy_r[0]*u->psi_r[0])=%f\n",(cg.dx_r+u->dy_r_LPF[0]*u->psi_r_LPF[0]));
	
    printf("(cg.dx_r)=%f\n",cgPtr->dx_r_LPF[0]);
    // printf("lemda2 = %f\n",lemda2);
	u->F_D1=D_1;
	u->F_ks1sat=u->ks1*saturation(u->SV1, epsilonSV1_U, epsilonSV1_L);
	u->F_lemda1SV1=u->lemda1*(u->SV1);
	u->F_dy_r_dx_r_curvature=u->dy_r_LPF[0]*cgPtr->dx_r_LPF[0]*d->curvature;

	u->M_1=-(2*(LF*Cyf-LR*Cyr)/IZ)*u->dy_r_LPF[0]/(cgPtr->VXLPF[0]);
	u->M_2=(2*(LF*Cyf-LR*Cyr)*u->psi_r[0]/IZ);
	u->M_3=-(2*(LF*LF*Cyf+LR*LR*Cyr)/IZ)*u->r_r[0]/(cgPtr->VXLPF[0]);
	u->M_D2=D_2;
	u->M_ks2sat=ks2*saturation(u->SV2, epsilonSV2_U, epsilonSV2_L);
	u->M_lemda2SV2=u->lemda2*u->SV2;


	// tracker calibration 
   // if(sec<6){
   //      u->FX = 120; //+MASS*1.5
   //      u->M = 0;
   //      //u->M = -IZ*(1*u->r_r[0]+1*u->psi_r[0]);
   //  }
   //  else{
   //   	u->FX = -100;
   // 		 u->M = 0;   	
   //  }



    return 1;

}


