﻿/*
 *  MotionEngine.c
 *
 *  
 *  wordt op 1 kHz, interrupt getriggerd
 *	De stappen en motorregeling  
 *
 *  Created: 10/04/2026
 *  Authors: Raph van Koeveringe (/ Robbe)
 */

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "MotionEngine.h"
#include "DeltaKinematics.h"

#include "Map.h" // voor constrain() and fmap()

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DAC4921Lib.h"
#include "QC7366Lib.h"

///////////////////////////////////////////////////////////////////////////////
// globals
double g_time = 0.0;               // time in seconds
uint64_t g_tickCount = 0;          // clock tick count from 1 ms hardware clock
uint16_t g_TicksPerSecond = 0;     // = 1 / sample time

static uint8_t motorIndex = 0;
static uint32_t encoderCounts = 0;
static double motorPos = 0;

///////////////////////////////////////////////////////////////////////////////
// general settings

const double Ts = 0.001;           // sample time, must match external clock
const double pi = 3.14159;

static const uint8_t MotorQcChannel[N_MOTORS] = {0, 1, 2};
static const uint8_t MotorDacChannel[N_MOTORS] = {0, 1, 2};

static const double EncoderCountsPerRevolution = 4096.0;
static const double itwk = 47.0;

///////////////////////////////////////////////////////////////////////////////
// controller settings

double maxVelocityUpperArm = 1.0;          // TODO: <Set maximum upper arm velocity [rad/s].>
double maxAccelerationUpperArm = 1.0;      // TODO: <Set maximum upper arm acceleration [rad/s^2].>

double Kp = 1.0;                           // TODO: <Tune proportional gain.>
double C2 = 1.0;                           // TODO: <Set motor/drive stiffness term.>
double JeqUpperArm = 0.0;                  // TODO: <Equivalent inertia seen in upper arm coordinates.>


double maxControlVoltage = 4.0;            // TODO: <Set absolute maximum motor voltage.>

///////////////////////////////////////////////////////////////////////////////
// position / motion state
static double motorControlOutput[N_MOTORS]	= {0.0, 0.0, 0.0};	// Berekende output
static double uDac[N_MOTORS]				= {0.0, 0.0, 0.0};	// Gelimiteerde uDac output [V] met uDac = Constraint(g,min,max)
	
static double angleError[N_MOTORS]			= {0.0, 0.0, 0.0};	// Berekende fout [rad]
static double actualUpperArmPos[N_MOTORS]	= {0.0, 0.0, 0.0};	// Gemeten arm positie [rad]

static uint32_t moveTick = 0;
static uint32_t moveTicksTotal = 0;



///////////////////////////////////////////////////////////////////////////////
// static void ReadUpperArmPositions(double measuredUpperArmPosition[N_MOTORS])
//
// Reads motor encoders and converts the motor rotor angle to upper arm angle.
// Horizontaal is nul-positie.

static double *ReadUpperArmPositions(void)
{
	motorIndex = 0;

	// Voor iedere motor
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		// encouderCount uitlezen en naar hoekpositie omrekenen
		encoderCounts = qc_ReadCountRegister(MotorQcChannel[motorIndex]);
		motorPos =((double)encoderCounts / EncoderCountsPerRevolution) * 2.0 * pi; //hoeveel radialen verplaatst t.o.v. (0, oftewel horizontaal)
		actualUpperArmPos[motorIndex] = (motorPos / itwk);
	}
	
	return actualUpperArmPos;
}



///////////////////////////////////////////////////////////////////////////////
// void HoldPosition(double q1, double q2, double q3)
//
// Wordt na iedere 1 ms tick opgeroepen. 
// behoud de alle bovenarmen op gewenste hoekposities.

double largeErrorThreshold		= 0.1;	// Wanneer fout groter is dan 0.1 [Rad]
double slowApproachVoltage		= 0.75;	// [V] Gewenste voltage voor langzame verplaatsing
static double holdUpperArmPos[N_MOTORS]		= {0.0, 0.0, 0.0};	// Vast te houden positie [rad]

void HoldPosition(double rad_arm1, double rad_arm2, double rad_arm3)
{
	motorIndex = 0;

	// Sla gewenste posities op
	holdUpperArmPos[0] = rad_arm1;
	holdUpperArmPos[1] = rad_arm2;
	holdUpperArmPos[2] = rad_arm3;

	// Lees bovenarmposities uit.
	actualUpperArmPos = ReadUpperArmPositions();

	// Voor iedere motor
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		// Fout bepalen
		angleError[motorIndex] = holdUpperArmPos[motorIndex] - actualUpperArmPos[motorIndex];
		
		// Voltage berekenen mbv PID_Controller
		motorControlOutput[motorIndex] = PID_Controller( angleError[motorIndex]) );

		//Als de fout extreem is, rustig naar referentie punt toe verplaatsen.
		if (fabs(angleError[motorIndex]) > largeErrorThreshold)
		{
			// Als fout positief is, voltage positief en visa versa.
			motorControlOutput[motorIndex] = (angleError[motorIndex] >= 0.0) ? slowApproachVoltage : -slowApproachVoltage;
		}

		// Zorg dat waarde niet groter is dan maximale DAC-waarde, en output.
		uDac[motorIndex] = constrain(motorControlOutput[motorIndex], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE); // min/max ongeveer +/-10V.
		dac_SetOutputVoltage(MotorDacChannel[motorIndex], motorControlOutput[motorIndex]);
	}
	
}//End HoldPosition();





///////////////////////////////////////////////////////////////////////////////
// bool RunSequence(void)
//
// Called once per 1 ms tick.
// Beslist welke stap er uitgevoerd moeten worden.
uint8_t stap = 0;
Bool sequenceDone = false;

bool RunSequence(void) 
{
	Move_ToSetpoint(100,100,-300, 1.1) // x,y,z,Tmax -> setpoint
	{
		sequenceDone = true;
		stap = 0;
	}
	return sequenceDone
}//end runSequence();


///////////////////////////////////////////////////////////////////////////////
// bool Move_ToSetpoint(void)
//
// Called once per 1 ms tick. Evaluates the move profile and controls the motors.
Bool setupMotionProfileDone = false;
static double EindPos[N_MOTORS]	= {0.0, 0.0, 0.0};	// Berekende output

static const double  xMax = 0.0; // [m] eind positie
static const double  vMax = 1.0; // [m/s] maximale snelheid
static const double  aMax = 1.0; // [m/s2] maximale acceleratie
static const double  rMax = NULL; // [m/s3] maximale RUK

static Bool MotionEnded = false;
static double t1, t2, tstart = 0.0;
static double alphag, omegag, thetag;

Bool Move_ToSetpoint(double x_eindPos, double y_eindPos, double z_eindPos, double Tmax)
{
	motorIndex = 0;
	
	// Uitlezen van actuele positie
	actualUpperArmPos = ReadUpperArmPositions();
	
	// Bepalen van (motorhoek) eindpositie.
	EindPos = DeltaKinematics_Inverse(double x_eindPos, double y_eindPos, double z_eindPos); // inversekinematica
	
	
	// Eenmalig nieuw bewegingsprofiel berekenen.
	if(!setupMotionProfileDone)
	{
		//double eindPos_M0, eindPos_M1, eindPos_M2 = NULL;

		
		//TODO: actuele grijper positie bepalen  (armPos -> grijperPos transformatie)
		//TODO: Te verplaatsen in X,Y,Z richting.
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			// totaal te verplaatsen hoekbepalen
			double path = actualUpperArmPos[motorIndex] - EindPos[motorIndex]
			
			double alpha0, alpha1, alpha2, alpha3, alphaMax = NULL;
			double rMax = path * 32.0/pow(Tmax, 3);	// maximale ruk [m/s3]



			// translatie bewegingsprofiel opstellen (positie derdegraads)
			alpha0 = 0.0;
			alpha1 = rMax*Tmax/4.0;
			alpha2 = -rMax*Tmax/4.0;
			alpha3 = 0.0;
	
			alphaMax	= rMax*Tmax/4.0;
	
			t1 = Tmax/4.0;
			t2 = 3.0*Tmax/4.0;
		}
		
		setupMotionProfileDone = true
	}
	
	
	
	
	// output voltage voor iedere motor bepalen t.o.v. referentiebewegingsprofiel
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		// opstellen van een referentiebewegingsprofiel voor de regeling
		// Zolang de huidige tijd nog voor het startmoment ligt:
		if (g_time <= 0)		// 1
		{
			alphag  = 0.0;
			omegag = 0.0;
			thetag = 0.0;
			MotionEnded = false;
		}
		// Eerste kwart van de beweging: versnelling opbouwen
		else if ( g_time <= (Tmax/4.0) + tstart )		//	2
		{
			alphag	= (4.0 * alphaMax/Tmax) * (g_time-tstart);			// feed forward
			omegag	= 2.0 * (alphaMax/Tmax) * pow(g_time-tstart, 2.0);	// feed forward
			thetag	= (2.0/3.0) * (alphaMax/Tmax) * pow(g_time-tstart, 3.0);
			
			MotionEnded = false;
		}
		// 3; van versnellen naar vertragen
		else if ( g_time <= (3.0*Tmax/4.0) + tstart )	//	3
		{
			alphag	=	alphaMax - 4.0*(alphaMax/Tmax)*(g_time-(t1+tstart));
			omegag	=	alphaMax * (g_time-(t1+tstart))
			-2.0 * (alphaMax/Tmax) * pow((g_time-(t1+tstart)), 2.0)
			+ (1.0/8.0)*alphaMax*Tmax;
			thetag	=	0.5 * alphaMax * pow( (g_time-(t1+tstart)), 2.0)
			- (2.0/3.0) * (alphaMax/Tmax) * pow( (g_time-(t1+tstart)), 3.0)
			+ (1.0/8.0)*alphaMax*Tmax*(g_time-(t1+tstart))
			+ (2.0/(3.0*64.0))*alphaMax*pow(Tmax, 2.0);
			
			MotionEnded = false;
		}
		// 4; van vertraging terug naar stilstand, actief remmen tot steeds minder remmen.
		else if ( g_time <= (Tmax + tstart) )
		{
			alphag	=	-alphaMax + 4.0*(alphaMax/Tmax) * (g_time-(t2+tstart));
			omegag	=	-alphaMax * (g_time-(t2+tstart))
			+ 2.0*(alphaMax/Tmax) * pow((g_time-(t2+tstart)), 2.0)
			+ (1.0/8.0)*alphaMax*Tmax;
			thetag	=	-(alphaMax/2.0) * pow((g_time-(t2+tstart)), 2.0)
			+ (2.0/3.0) * (alphaMax/Tmax) * pow( (g_time-(t2+tstart)), 3.0 )
			+ (1.0/8.0) * alphaMax*Tmax*(g_time-(t2+tstart))
			+ (22.0/(3.0*64.0)) * alphaMax*pow(Tmax, 2.0);
			MotionEnded = false;
		}
		//	5; op eindpunt en Tmax verstreken.
		else
		{
			alphag	=	0.0;
			omegag	=	0.0;
			thetag	=	EindPos[motorIndex];
			MotionEnded = true;
			setupMotionProfileDone = false;
		}

		angleError[motorIndex] = alphag - actualUpperArmPos[motorIndex]
		uDac[motorIndex] = PID_Controller(angleError[motorIndex]) + FeedForward(alphag);
		uDac[motorIndex] = constrain(uDac[motorIndex], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE)
	}
	
	// tegelijk Daadwerkelijk iedere motor outputvoltage zetten.
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		dac_SetOutputVoltage(motorIndex, uDac[motorIndex]);
	}
	
	return MotionEnded;
}//END-function