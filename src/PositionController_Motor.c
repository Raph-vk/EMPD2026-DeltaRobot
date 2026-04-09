/*
 *  PositionController.c
 *
 *  Created: 14-9-2023 11:42:11
 *  Authors: Raoul Smeets / Hans Langen
 */ 


///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DAC4921Lib.h"
#include "QC7366Lib.h"

///////////////////////////////////////////////////////////////////////////////
// controller includes

#include "PositionController.h"
#include "map.h"

///////////////////////////////////////////////////////////////////////////////
// globals

double g_time = 0.0;				// time in seconds
uint64_t g_tickCount = 0;			// clock tick count from 1 ms hardware clock
uint16_t g_TicksPerSecond = 0.0;


///////////////////////////////////////////////////////////////////////////////
// Sample and Hold

///////////////////////////////////////////////////////////////////////////////

const double pi		=	3.14159;	// guess what ;-)
const double rhoAlu	=	2700;		// dichtheid aluminium
const double rhoSUS	=	7800;		// dichtheid staal

///////////////////////////////////////////////////////////////////////////////

const double Ts			= 0.001;	// must match external clock!!
const double tstart		= 0.1;		// start bewegingsprofiel

///////////////////////////////////////////////////////////////////////////////
// mechanische parameters

///////////////////////////////////////////////////////////////////////////////
// torsiestijfheid

const double kspec	=	2.8e5;		// specifiek stijfheid [N] -AT5 tandriem 16mm breed met ijzerdraad
const double Ltr	=	0.4;		// afstand tussen poelie 1 en 2 [m]
double Lac			=	0.0;		// lengte tandriem
double ktr			=	0.0;		// lin. stijfheid tandriem
double K12			=	0.0;		// torsiestijfheid voorgespannen tandriem

///////////////////////////////////////////////////////////////////////////////
// massa en massatraagheden

const double mL1	=	0.5;		// massa slede (moterkant) [kg]
const double mL2	=	0.5;		// massa slede (lastkant) [kg]
const double d1		=	0.02;		// dikte poelie [m]

double mp1			=	0.0;
double Jp1			=	0.0;		// massatraagheid poelie 1 [kgm2]
const double d2		=	0.02;
double mp2			=	0.0;
double Jp2			=	0.0;		// massatraagheid poelie 2
const double Ras	=	0.006;
const double Las	=	0.02;
double mas			=	0.0;
double Jas			=	0.0;		// massatraagheid reductoras [kgm2]


///////////////////////////////////////////////////////////////////////////////
// overbrengingsverhoudingen

const double R1		=	0.033;		// radius poelie1
const double R2		=	0.033;		// radius poelie2
double i1			=	0.0;		// rotatie naar lineaire verplaatsing
double i2			=	0.0;		// rotatie naar lineaire verplaatsing
const double i0fh	=	6.6;		// overbrengingsverh. Faulhaberoverbr.
double itot			=	0.0;


///////////////////////////////////////////////////////////////////////////////
// transformeren, inertia match

double Jstar		=	0.0;
double Je1			=	0.0;
double Je2			=	0.0;
double war			=	0.0;		// antiresonantie freq. [rad/s]
double mu			=	0.0;
double wr			=	0.0;		// resonantie freq. [rad/s]
const double betaopt=	0.8;		// servodemp.+ derdegraads bewegingsprofiel


///////////////////////////////////////////////////////////////////////////////
// motordata

const double Jm		=	63e-7;		// massatraagheid motoras Faulhaber
const double Rm		=	1.47;		// weerstand motor [Ohm]
const double Lind	=	110e-6;		// inductie motor [Henry]
const double Kt		=	0.0435;		// motorconstante [Nm/A]


///////////////////////////////////////////////////////////////////////////////
// controller + versterker

const double Ka		=	0.2;		// spanning naar stroomversterker [A/V]
const double C1		=	1.0;		// Kpm*C1,de regelaar, met C1=1 een P-
double C2			=	0.0;        // Kpm*C2 is de servostijfheid

///////////////////////////////////////////////////////////////////////////////
// regelaarinstelling 

double wbm		=	0.0;			// bandwidth [rad/s]
double wbl		=	0.0;
double taudm	=	0.0;
double tauim	=	0.0;
double taufm	=	0.0;
double Ksm		=	0.0;
double Kpm		=	0.0;

//U(z)/E(z)=Kp*[p0+p1*z^(-1)+p2*z^(-2)]/[q0+q1*z^(-1)+q2*z^(-2)]

double p0		=	0.0;
double p1		=	0.0;
double p2		=	0.0;
double q0		=	0.0;
double q1		=	0.0;
double q2		=	0.0;

///////////////////////////////////////////////////////////////////////////////
// bewegingsprofiel (derdegraads)

const double xmax		=	0.15;		// max verplaatsing [m]
double tmax				=	0.0;		// max opzettijd [s]
double fiemax			=	0.0;		// max hoekverplaatsing motor
double jmax				=	0.0;		// max jerk
double alfamax			=	0.0;

double t3[5]			=	{0.0, 0.0, 0.0, 0.0, 0.0};
double alfa3[5]			=	{0.0, 0.0, 0.0, 0.0, 0.0};
	
double t1				=	0.0;
double t2				=	0.0;

double Je1m				=	0.0;
double Je2m				=	0.0;


///////////////////////////////////////////////////////////////////////////////
// variables

double theta1				=	0.0;
double theta1_window[4]		=	{0.0, 0.0, 0.0, 0.0};
double uth1_window[4]		=	{0.0, 0.0, 0.0, 0.0};
double error_th1_window[4]	=	{0.0, 0.0, 0.0, 0.0};

// gew. hoekposities, -snelheid & -versnelling voor motoras

double thetag	=	0.0;
double omegag	=	0.0;
double alfag	=	0.0;
double ffdM		=	0.0;	// feed forward


///////////////////////////////////////////////////////////////////////////////
// void posctrl_InitParameters(double wbmFactor)

void posctrl_InitParameters(double wbmFactor)
{
	g_TicksPerSecond = 1.0/Ts;

	// ******
	// IMPORTANT: after each restart, the time for the controller must be reset
	// by setting g_tickCount to zero !!! See next line.
	// ******
	
	g_tickCount = 0;	// reset the time

	Lac = sqrt( pow(Ltr, 2) + pow(R2-R1, 2) );
	ktr = kspec/Lac;
	K12 = 2 * ktr * pow(R1,2);				// torsiestijfheid voorgespannen tandriem
	
	mp1 = pi * pow(R1, 2) * d1 * rhoAlu;
	Jp1 = 0.5 * mp1 * pow(R1, 2);			// massatraagheid poelie 1 [kgm2]
	mp2 = pi * pow(R2, 2) * d2 * rhoAlu;
	Jp2 = 0.5 * mp2 * pow(R2, 2);			// massatraagheid poelie 2
	
	mas = pi * pow(Ras, 2) * Las *rhoSUS;
	Jas = 0.5 * mas * pow(Ras, 2);		//massatraagheid reductoras [kgm2]
	
	i1	 = 1.0/R1;
	i2	 = R2;	
	itot = i0fh*i1;
	
	//transformeren, inertia match
	Jstar	= Jas + Jp1 + mL1/pow(i1, 2) + mL2/pow(i1, 2) + Jp2/(pow(i2, 2) * pow(i1, 2));
	Je1		= Jm*pow(i0fh, 2) + Jas + Jp1 + mL1/pow(i1, 2);
	Je2		= mL2/pow(i1, 2) + Jp2/(pow(i1, 2) * pow(i2, 2)) ;  
	
	// is this correct???? Hans???
	Je1m	= Je1/pow(i0fh, 2);
	Je2m	= Je2/pow(i0fh, 2);
	
	war		= sqrt(K12/Je2);				// antiresonantie freq. [rad/s]
	mu		= (Je1*Je2)/(Je1+Je2);
	wr		= sqrt(K12/mu);					// resonantie freq. [rad/s]

	C2		= Ka*Kt;						// Kpm*C2 is servostijfheid

	//wbm = 0.2 * wr;
	//wbm = 0.025 * wr;	// test
	
	wbm = wbmFactor * wr;
	wbl =  0.1 * wr;
	
	taudm = 1.0/(wbm/3.0);
	tauim = 1.0/(wbm/10.0);
	taufm = 1.0/(3.0*wbm);

	Ksm = pow(wbm, 2) * (Je1m+Je2m);
	Kpm = Ksm/C2;

	// U(z)/E(z)=Kp*[p0+p1*z^(-1)+p2*z^(-2)]/[q0+q1*z^(-1)+q2*z^(-2)]
	
	p0 = 1.0 + (2.0/Ts) * (taudm+tauim) + 4.0*taudm*tauim/pow(Ts, 2);
	p1 = 2.0 * (1.0-4.0*taudm*tauim/pow(Ts,2));
	p2 = 1.0 - (2.0/Ts)*(taudm+tauim) + 4.0*taudm*tauim/pow(Ts, 2);
	q0 = (2.0*tauim/Ts) * (1.0+2.0*taufm/Ts);
	q1 = -8.0 * taufm * tauim/pow(Ts, 2);
	q2 = (2.0*tauim/Ts) * (-1.0+2.0*taufm/Ts);

	// bewegingsprofiel (derdegraads)
	
	//tmax	=	5*2*pi/(0.3*wr);			// max opzettijd [s]
	tmax	=	5.0 * 2.0 * pi/(1.0*wbm);	// max opzettijd [s] - TEST
	fiemax	=	xmax*itot;					// max hoekverplaatsing motor
	jmax	=	fiemax * 32.0/pow(tmax, 3);	// max jerk

	t3[1]	= 0.0;
	t3[2]	= tmax/4.0;
	t3[3]	= 3.0 * tmax/4.0;
	t3[4]	= tmax;
	
	alfa3[1] = 0.0;
	alfa3[2] = jmax*tmax/4.0;
	alfa3[3] = -jmax*tmax/4.0;
	alfa3[4] = 0.0;
	
	alfamax	= jmax*tmax/4.0;
	
	t1 = tmax/4.0;
	t2 = 3.0*tmax/4.0;
}


///////////////////////////////////////////////////////////////////////////////
// void posctrl_RunController_MotorSide(void)

void posctrl_RunController_MotorSide(void)
{
	uint8_t qcChannel  = 1;		// werkend op regeling MOTORKANT = QC kanaal 1 !!
	uint8_t dacChannel = 0;
	int32_t qcCount	   = 0;
	double  dacOutputVoltage = 0.0;
	
	if (g_time <= tstart)		// 1
	{
		alfag  = 0.0;
		omegag = 0.0;
		thetag = 0.0;
	}
	else if ( g_time <= (tmax/4.0) + tstart )		//	2
	{
		alfag	= (4.0 * alfamax/tmax) * (g_time-tstart);			// feed forward
		omegag	= 2.0 * (alfamax/tmax) * pow(g_time-tstart, 2.0);	// feed forward
		thetag	= (2.0/3.0) * (alfamax/tmax) * pow(g_time-tstart, 3.0);
		
	}
	else if ( g_time <= (3.0*tmax/4.0) + tstart )	//	3
	{
		alfag	=	alfamax - 4.0*(alfamax/tmax)*(g_time-(t1+tstart));
		omegag	=	alfamax * (g_time-(t1+tstart))
					-2.0 * (alfamax/tmax) * pow((g_time-(t1+tstart)), 2.0) + (1.0/8.0)*alfamax*tmax;
		thetag	=	0.5 * alfamax * pow( (g_time-(t1+tstart)), 2.0)
					- (2.0/3.0) * (alfamax/tmax) * pow( (g_time-(t1+tstart)), 3.0)
					+ (1.0/8.0)*alfamax*tmax*(g_time-(t1+tstart)) + (2.0/(3.0*64.0))*alfamax*pow(tmax, 2.0);
	}
	else if ( g_time <= (tmax + tstart) )			//	4
	{
		alfag	=	-alfamax + 4.0*(alfamax/tmax)*(g_time-(t2+tstart));
		omegag	=	-alfamax * (g_time-(t2+tstart))
					+ 2.0*(alfamax/tmax) * pow((g_time-(t2+tstart)), 2.0) + (1.0/8.0)*alfamax*tmax;
		thetag	=	-(alfamax/2.0) * pow((g_time-(t2+tstart)), 2.0)
					+ (2.0/3.0) * (alfamax/tmax) * pow( (g_time-(t2+tstart)), 3.0 )
					+ (1.0/8.0) * alfamax*tmax*(g_time-(t2+tstart)) + (22.0/(3.0*64.0))*alfamax*pow(tmax, 2.0);
	}
	else	//	5
	{
		alfag	=	0.0;
		omegag	=	0.0;
		thetag	=	fiemax;
	}
	
	//------ inlezen encoder (hoekpositie motor)-------------------------
	// theta1 =(getencoder(motor, ...)/4096)*0.0314; //in meters
	
	qcCount = qc_ReadCountRegister(qcChannel);
	qcCount = -qcCount;
	theta1	= (qcCount / 4096.0) * 2.0 * pi;
	
	theta1_window[3] = theta1_window[2];
	theta1_window[2] = theta1_window[1];
	theta1_window[1] = theta1;
	//
	error_th1_window[3] = error_th1_window[2];
	error_th1_window[2] = error_th1_window[1];
	error_th1_window[1] = thetag-theta1;
	//
	uth1_window[3] = uth1_window[2];
	uth1_window[2] = uth1_window[1];

	float uDac = 0.0;
	
	// bewegingsprofiel op DAC channel 2
	uDac = fmap(thetag, 0.0, xmax*itot,  0.0, DAC_MAX_OUTPUTVOLTAGE);
	dacChannel = 2;
	dac_SetOutputVoltage(dacChannel, uDac);
	
	// error signaal op DAC channel 3
	uDac = fmap(error_th1_window[1], -xmax*itot, xmax*itot,  DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	dacChannel = 3;
	dac_SetOutputVoltage(dacChannel, uDac);
	
	//--- motorregeling met feedforward, uth1=(motor)spanning [V]
	
	ffdM = (Je1m+Je2m) * (alfag/C2);
	uth1_window[1] = (Kpm/q0)*(p0*error_th1_window[1]+p1*error_th1_window[2]+p2*error_th1_window[3])-(q1/q0)*uth1_window[2]-(q2/q0)*uth1_window[3];

	//------ Uitschrijven van spanningswaarden naar motordrive
	// setDAC(uth1_window[1]+ffdM);
	
	// motorspanning op DAC channel 0
	dacOutputVoltage = uth1_window[1] + ffdM;
	dacChannel = 0;
	dac_SetOutputVoltage(dacChannel, dacOutputVoltage);
	
	g_tickCount++;
	g_time = ((double)g_tickCount) / g_TicksPerSecond;
	
	if ((g_tickCount % 1000) == 0)
	{
		vPrintString(".");
	}
	
}
