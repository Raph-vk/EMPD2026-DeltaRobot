/*
 *  PositionControllerLoad.c
 *
 *	Regeling op lastzijde
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

#include "PositionControllerLoad.h"
#include "map.h"

///////////////////////////////////////////////////////////////////////////////
// globals

double g_time = 0.0;				// time in seconds
uint64_t g_tickCount = 0;			// clock tick count from 1 ms hardware clock
uint16_t g_TicksPerSecond = 0.0;	// = 1 / sampletime


///////////////////////////////////////////////////////////////////////////////
// Sample and Hold

///////////////////////////////////////////////////////////////////////////////

const double pi		=	3.14159;	// guess what ;-)
const double rhoAlu	=	2700;		// dichtheid aluminium
const double rhoSUS	=	7800;		// dichtheid staal

///////////////////////////////////////////////////////////////////////////////

const double Ts			= 0.001;	// sample time, must match external clock!!
double tstart			= 0.1;		// start bewegingsprofiel

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
double Je1L			=	0.0;
double Je2L			=	0.0;
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

double wbl			=	0.0;
double KsdL			=	0.0;
double KpsdL		=	0.0;
const double D12	=	0.0;
double D12L			=	0.0;
double KvmL			=	0.0;
double KtaL			=	0.0;

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

///////////////////////////////////////////////////////////////////////////////
// variables

double xL2					=	0.0;
double vL1					=	0.0;

double theta1				=	0.0;
double theta1_window[4]		=	{0.0, 0.0, 0.0, 0.0};	// nu gebruiken voor lastsnelheid
double uxL2_window			=	0.0;
double xL2_window			=	0.0;
double error_xL2_window		=	0.0;

double vL1_window[4]		=	{0.0, 0.0, 0.0, 0.0};
	
// gew. hoekposities, -snelheid & -versnelling voor motoras

double thetag	=	0.0;
double omegag	=	0.0;
double alfag	=	0.0;


///////////////////////////////////////////////////////////////////////////////
//void posctrlLoad_InitParameters(double wblFactor)
// Mechanische parameters bepalen en instellen.
void posctrlLoad_InitParameters(double wblFactor)
{
	g_TicksPerSecond = 1.0/Ts;				// must match external clock!!

	// ******
	// IMPORTANT: after each restart, the time for the controller must be reset
	// by setting g_tickCount to zero !!! See next line.
	// ******

	g_tickCount = 0;	// reset the time
	
	tstart = 0.1;
	
	// RIEM mechanische parameters
	Lac = sqrt( pow(Ltr, 2) + pow(R2-R1, 2) ); // Lengte riem bepalen
	ktr = kspec/Lac; // lin. stijfheid tandriem
	K12 = 2.0 * ktr * pow(R1, 2);			// torsiestijfheid voorgespannen tandriem
	
	// Pulley
	mp1 = pi * pow(R1, 2) * d1 * rhoAlu;	// massa poelie 1 bepalen
	Jp1 = 0.5 * mp1 * pow(R1, 2);			// massatraagheid poelie 1 [kgm2]
	mp2 = pi * pow(R2, 2) * d2 * rhoAlu;    // massa poelie 2 bepalen
	Jp2 = 0.5 * mp2 * pow(R2, 2);			// massatraagheid poelie 2
	
	mas = pi * pow(Ras, 2) * Las *rhoSUS;
	Jas = 0.5 * mas * pow(Ras, 2);		//massatraagheid reductoras [kgm2]
	
	// Overbrengingsverhoudingen
	i1	 = 1.0/R1;	// [R->T]
	i2	 = R2;		// [T->R]
	itot = i0fh*i1; // [R->T] i0fh=R-R tandwielkast
	
	//transformeren, inertia match, totale eq-massatraagheid gevoeld op de motor
	Jstar	= Jas + Jp1 + mL1/pow(i1, 2) + mL2/pow(i1, 2) + Jp2/(pow(i2, 2) * pow(i1, 2));
	
	//Equivalente massatraagheid naar de motorgerekend
	Je1		= Jm*pow(i0fh, 2) + Jas + Jp1 + mL1/pow(i1, 2);		// motorzijde
	Je2		= mL2/pow(i1, 2) + Jp2/(pow(i1, 2) * pow(i2, 2));	// lastzijde
	
	// naar de equivalente massatraagheid aan de lastzijde gerekend
	Je1L	= Je1 * pow(i1, 2);
	Je2L	= Je2 * pow(i1, 2);

	
	war		= sqrt(K12/Je2);				// antiresonantie freq. [rad/s]
	mu		= (Je1*Je2)/(Je1+Je2);
	wr		= sqrt(K12/mu);					// resonantie freq. [rad/s]
	
	C2		= Ka*Kt;						// Kpm*C2 is servostijfheid

	
	wbl		=  wblFactor * 0.1 * wr;
	
	KsdL	= pow(wbl, 2) * (Je1L + Je2L);
	KpsdL	= KsdL / C2;
	
	D12L = D12 * pow(i1, 2);
	KvmL = 2.0 * betaopt * sqrt(KsdL*(Je1L + Je2L)) - D12L;
	KtaL = KvmL/C2;

	// bewegingsprofiel (derdegraads)
	tmax	=	1.0 * 2.0 * pi/(0.3*wbl);	// max opzettijd [s] - TEST
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
// void posctrlLoad_RunController(void)
// 
void posctrlLoad_RunController(void)
{
	const uint8_t qcChannelLast		= 0;		// lastkant / Renishaw	= QC kanaal 0
	const uint8_t qcChannelMotor	= 1;		// motorkant			= QC kanaal 1
	const uint8_t dacChannelMotor	= 0;
	int32_t qcCount					= 0;
	double  dacOutputVoltage		= 0.0;
	
	// opstellen van een referentiebewegingsprofiel voor de regeling
	// Zolang de huidige tijd nog vóór het startmoment ligt:
	if (g_time <= tstart)		// 1
	{
		alfag  = 0.0;
		omegag = 0.0;
		thetag = 0.0;
	}
	// Eerste kwart van de beweging: versnelling opbouwen
	else if ( g_time <= (tmax/4.0) + tstart )		//	2
	{
		alfag	= (4.0 * alfamax/tmax) * (g_time-tstart);			// feed forward
		omegag	= 2.0 * (alfamax/tmax) * pow(g_time-tstart, 2.0);	// feed forward
		thetag	= (2.0/3.0) * (alfamax/tmax) * pow(g_time-tstart, 3.0);
	}
	// 3; van versnellen naar vertragen
	else if ( g_time <= (3.0*tmax/4.0) + tstart )	//	3
	{
		alfag	=	alfamax - 4.0*(alfamax/tmax)*(g_time-(t1+tstart));
		omegag	=	alfamax * (g_time-(t1+tstart))
					-2.0 * (alfamax/tmax) * pow((g_time-(t1+tstart)), 2.0) 
					+ (1.0/8.0)*alfamax*tmax;
		thetag	=	0.5 * alfamax * pow( (g_time-(t1+tstart)), 2.0)
					- (2.0/3.0) * (alfamax/tmax) * pow( (g_time-(t1+tstart)), 3.0)
					+ (1.0/8.0)*alfamax*tmax*(g_time-(t1+tstart)) 
					+ (2.0/(3.0*64.0))*alfamax*pow(tmax, 2.0);
	}
	// 4; van vertraging terug naar stilstand, actief remmen tot steeds minder remmen.
	else if ( g_time <= (tmax + tstart) )
	{
		alfag	=	-alfamax + 4.0*(alfamax/tmax) * (g_time-(t2+tstart));
		omegag	=	-alfamax * (g_time-(t2+tstart))
					+ 2.0*(alfamax/tmax) * pow((g_time-(t2+tstart)), 2.0) 
					+ (1.0/8.0)*alfamax*tmax;
		thetag	=	-(alfamax/2.0) * pow((g_time-(t2+tstart)), 2.0)
					+ (2.0/3.0) * (alfamax/tmax) * pow( (g_time-(t2+tstart)), 3.0 )
					+ (1.0/8.0) * alfamax*tmax*(g_time-(t2+tstart)) 
					+ (22.0/(3.0*64.0)) * alfamax*pow(tmax, 2.0);
	}
	//	5; op eindpunt blijvenstaan.
	else	
	{
		alfag	=	0.0;
		omegag	=	0.0;
		thetag	=	fiemax;
	}
	
	//------ inlezen encoder: verplaatsing Renishaw ----------
	// lineaire encoder op de last
	qcCount = qc_ReadCountRegister(qcChannelLast);	// Renishaw
	qcCount = -qcCount;
	xL2		= qcCount * 10e-6;	// in meters
	
	//------ inlezen encoder: hoekpositie motor ----------
	// rotatie encoder op de motor
	qcCount = qc_ReadCountRegister(qcChannelMotor);	// motor
	qcCount = -qcCount;
	theta1	= (qcCount / 4096.0) * 2.0 * pi;
	
	
	// laatste hoekpositie metingen opslaan en doorschuiven
	theta1_window[3] = theta1_window[2];
	theta1_window[2] = theta1_window[1];
	theta1_window[1] = theta1;
	
	// oude snelheid waarde doorschuiven en nieuwe hoeksnelheidswaarde bepalen
	vL1_window[2] = vL1_window[1];
	vL1_window[1] = (theta1_window[1] - theta1_window[2]) / Ts;
	vL1 = ((vL1_window[1] + vL1_window[2])/2.0) / itot; // lasttranslatiesnelheid bepalen.	
	
	// verplaatsing opslaan en error opslaan
	xL2_window = xL2;
	error_xL2_window = (thetag/itot) - xL2;
	
	//--- P-regelaar met feedforward en servodemping,
	//--- uxL2_window/itot=(motor)spanning [V]
	//			  (kp met fout verrekenen)+(servodemping)+( feedforward)
 	uxL2_window = (KpsdL*error_xL2_window) - (KtaL*vL1) + ((Je1L+Je2L)*(alfag/itot)/C2);

	//--- einde lastregeling

	float uDac = 0.0;
	uint8_t dacChannel = 0;
	
	// bewegingsprofiel op DAC channel 2
	uDac = fmap(thetag, 0.0, xmax*itot,  0.0, DAC_MAX_OUTPUTVOLTAGE);
	dacChannel = 2;
	dac_SetOutputVoltage(dacChannel, uDac);
	
	// error signaal op DAC channel 3
	//uDac = fmap(error_th1_window[1], -xmax*itot, xmax*itot,  DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	//dacChannel = 3;
	//dac_SetOutputVoltage(dacChannel, uDac);
		
	// motorspanning op DAC channel 0
	dacOutputVoltage = uxL2_window/itot;
	dac_SetOutputVoltage(dacChannelMotor, dacOutputVoltage);
	
	g_tickCount++;
	g_time = ((double)g_tickCount) / g_TicksPerSecond;
	
	if ((g_tickCount % 1000) == 0)
	{
		vPrintString(".");
	}
	
}
