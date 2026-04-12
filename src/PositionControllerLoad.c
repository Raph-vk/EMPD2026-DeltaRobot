/*
 *  PositionControllerLoad.c
 *
 *  Position control on the motor side.
 *  Rough implementation lines for hold and move control on a 1 ms tick.
 *
 *  Created: 12/04/2026
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

//#include "MotorControl.h"
#include "PositionControllerLoad.h"

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
		
		// Voltage berekenen
		motorControlOutput[motorIndex] = (Kp * angleError[motorIndex]); // output [V] = Kp [V/rad] * error [rad] (wellicht nog D en Iactie toevoegen?S

		//Als te fout nog extreem is, rustig aan naar toe verplaatsen.
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
	if ((stap == 0) &&		MoveTo(0.0, 0.0, -200.0, 1.1))		stap = 1;
	else if ((stap == 1) && MoveTo(-100.0, 100.0, -200.0, 3.0))	stap = 2;
	else if ((stap == 2) && MoveTo(100.0, -100.0, -200.0, 1.1))
	{
		sequenceDone = true;
		stap = 0;
	}
	return sequenceDone
}//end runSequence();


///////////////////////////////////////////////////////////////////////////////
// bool MoveTo(void)
//
// Called once per 1 ms tick. Evaluates the move profile and controls the motors.
Bool setupMotionProfileDone = false;
static double grijperEindPos[N_MOTORS]	= {0.0, 0.0, 0.0};	// Berekende output

static const double  xMax = 0.0; // [m]
static const double  vMax = 1.0; // [m/s]
static const double  aMax = 1.0; // [m/s2]
static const double  aMax = 1.0; // [m/s2]




Bool MoveTo(double x_eindPos, double y_eindPos, double z_eindPos, double Tmax)
{
	motorIndex = 0;
	
	// Uitlezen van actuele positie
	actualUpperArmPos = ReadUpperArmPositions();
	

	
	
	// Eenmalig nieuw bewegingsprofiel berekenen.
	if(!setupMotionProfileDone)
	{
		// Sla gewenste posities op
		grijperEindPos[0] = x_eindPos;
		grijperEindPos[1] = y_eindPos;
		grijperEindPos[2] = z_eindPos;
		
		//TODO: actuele grijper positie bepalen  (armPos -> grijperPos transformatie)
		//TODO: Te verplaatsen in X,Y,Z richting.
		
		double rMax = x_tomove * 32.0/pow(tmax, 3);	// maximale ruk [m/s3]


		// translatie bewegingsprofiel opstellen (positie derdegraads)
		t_cycle[1]	= 0.0;
		t_cycle[2]	= tmax/4.0;
		t_cycle[3]	= 3.0 * tmax/4.0;
		t_cycle[4]	= tmax;
	
		alfa3[1] = 0.0;
		alfa3[2] = jmax*tmax/4.0;
		alfa3[3] = -jmax*tmax/4.0;
		alfa3[4] = 0.0;
	
		alfamax	= jmax*tmax/4.0;
	
		t1 = tmax/4.0;
		t2 = 3.0*tmax/4.0;
		
		
		setupMotionProfileDone = true
	}
	
	
	
	
	


	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		deltaPosition =
		moveGoalUpperArmPosition[motorIndex] - moveStartUpperArmPosition[motorIndex];

		referenceUpperArmPosition[motorIndex] =
		moveStartUpperArmPosition[motorIndex] + (s * deltaPosition);

		if (moveDurationSeconds > 0.0)
		{
			sSecond = (6.0 - (12.0 * tau)) / (moveDurationSeconds * moveDurationSeconds);
			referenceUpperArmAcceleration[motorIndex] = deltaPosition * sSecond;
		}
		else
		{
			referenceUpperArmAcceleration[motorIndex] = 0.0;
		}

		upperArmPositionError[motorIndex] =
		referenceUpperArmPosition[motorIndex] - actualUpperArmPosition[motorIndex];

		feedforwardVoltage = (JeqUpperArm * referenceUpperArmAcceleration[motorIndex]) / C2;

		motorControlOutput[motorIndex] =
		(Kp * upperArmPositionError[motorIndex]) + feedforwardVoltage;

		voltageLimit = maxControlVoltage;
		if (fabs(upperArmPositionError[motorIndex]) > largeErrorThreshold)
		{
			voltageLimit = slowApproachVoltageLimit;
		}

		motorControlOutput[motorIndex] =
		constrain(motorControlOutput[motorIndex], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);

		dac_SetOutputVoltage(MotorDacChannel[motorIndex], motorControlOutput[motorIndex]);
	}

	if (moveTick < moveTicksTotal)
	{
		moveTick++;
	}

	g_tickCount++;
	g_time = ((double)g_tickCount) * Ts;

	if (moveTick >= moveTicksTotal)
	{
		moveActive = false;

		holdUpperArmPosition[0] = moveGoalUpperArmPosition[0];
		holdUpperArmPosition[1] = moveGoalUpperArmPosition[1];
		holdUpperArmPosition[2] = moveGoalUpperArmPosition[2];

		return true;
	}

	return false;
}