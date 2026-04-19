/*
 *  MotionEngine.c
 *  
 *  MotionEngine for controlling the delta robot's motors to follow a desired trajectory.
 *
 *  Created: 10/04/2026
 *  Authors: Raph van Koeveringe (/ Robbe)
 */
///////////////////////////////////////////////////////////////////////////////
#include "MotionEngine.h"
///////////////////////////////////////////////////////////////////////////////
// globals
static const float Ts = 0.001f;      // sample time, must match external clock

static float g_time = 0.0f;          // totale tijd sinds opstarten van de robot in seconden
static float holdTargetPos[N_MOTORS] = {0.0f, 0.0f, 0.0f};
static Bool holdTargetValid = false;

///////////////////////////////////////////////////////////////////////////////
// general settings
static uint8_t mI = 0; //Motor index
static const float EncoderCountsPerRevolution = 2048.0f; // 2048 counts per revolution of the motor shaft

///////////////////////////////////////////////////////////////////////////////
// position control variables
static float motorControlOutput[N_MOTORS]	= {0.0, 0.0, 0.0};	// Berekende output
static float uDac[N_MOTORS]				= {0.0, 0.0, 0.0};	// Gelimiteerde uDac output [V] met uDac = Constraint(g,min,max)
	
static float Fout_motorRad[N_MOTORS]			= {0.0f, 0.0f, 0.0f};	// Berekende fout in motor-rad
static float motorPos_Rad[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Gemeten motor-as positie [rad]
static uint8_t stap = 0;
static Bool sequenceDone = false;
static Bool setupMotionProfileDone = false;
static Bool verplaatsingKlaar = false;

static const float degToRad = 0.01745329251994329576923690768489f; //PI / 180.0f; // conversiefactor van graden naar radialen
static const float RadToDeg = 57.295779513082320876798154814105f; // 180.0f / PI; // conversiefactor van radialen naar graden

///////////////////////////////////////////////////////////////////////////////
// High-level entry points expected by ControlTask

void MotionEngine_Init(void)
{
	g_time = 0.0f;
	stap = 0;
	sequenceDone = false;
	setupMotionProfileDone = false;
	verplaatsingKlaar = false;
	holdTargetValid = false;
}

void MotionEngine_HoldCurrentPosition(void)
{
	ReadMotorPositions(motorPos_Rad);

	for (mI = 0; mI < N_MOTORS; mI++)
	{
		holdTargetPos[mI] = (motorPos_Rad[mI] / 47.0f) / RadToDeg; // Omzetten naar gewenste armhoek in graden, rekening houdend met reductie.
	}
	holdTargetValid = true;
	HoldPosition(holdTargetPos);
}

Bool MotionEngine_RunSequence(void)
{
	holdTargetValid = false;
	return RunSequence();
}

void MotionEngine_Disable(void)
{
	MotionEngine_ResetSequence();

	for (mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], 0.0f);
	}
}

///////////////////////////////////////////////////////////////////////////////
// bool RunSequence(void)
//
// Called once per 1 ms tick.
// Beslist welke stap er uitgevoerd moeten worden.

Bool RunSequence(void)
{
	if (Move_ToSetpoint(100,100,-300, 1.1)) // x,y,z,Tmax -> setpoint
	{
		sequenceDone = true;
		stap = 0;
	}

	return sequenceDone;
}//end runSequence();


///////////////////////////////////////////////////////////////////////////////
// void HoldPosition(array HoekBoven)
//
// Wordt na iedere 1 ms tick opgeroepen.
// Behoudt alle motorassen op gewenste motorhoeken.
static const float largeErrorThreshold		= 0.1f;	// Wanneer fout groter is dan 0.1 motor-rad
static const float slowApproachVoltage		= 0.75f;	// [V] Gewenste voltage voor langzame verplaatsing
static float holdMotorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};// Vast te houden motorpositie [rad]
Bool nearReference = true;

Bool HoldPosition(const float holdArmPos_DegInput[N_MOTORS])
{
	mI = 0;
	
	// Lees motorposities uit.
	ReadMotorPositions(motorPos_Rad);

	// Voor iedere motor
	for (mI = 0; mI < N_MOTORS; mI++)
	{
 		// Gewenste armpositie omzetten naar radialen en rekening houden met reducties		
		holdMotorPos_Rad[mI] = i_twk * (holdArmPos_DegInput[mI] * degToRad);

		// Fout bepalen
		Fout_motorRad[mI] = holdMotorPos_Rad[mI] - motorPos_Rad[mI];

		//Als de fout extreem is, rustig naar referentie punt toe verplaatsen.
		if (fabsf(Fout_motorRad[mI]) > largeErrorThreshold)
		{
			// Als fout positief is, voltage positief en visa versa.
			motorControlOutput[mI] = (Fout_motorRad[mI] >= 0.0f) ? slowApproachVoltage : -slowApproachVoltage;
			nearReference = false;
		}
		else
		{
			// Voltage berekenen mbv PID_Controller
			motorControlOutput[mI] = PID_Controller( Fout_motorRad[mI] );
		}

		// Zorg dat waarde niet groter is dan maximale DAC-waarde, en output.
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE); // min/max ongeveer +/-10V.
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}

	return nearReference;
}//End HoldPosition();

///////////////////////////////////////////////////////////////////////////////
// bool Move_ToSetpoint(void)
//
// Called once per 1 ms tick. Evaluates the move profile and controls the motors.
static float motorTargetRad[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Berekende motor-eindpositie [rad]

static const float  xMax = 0.0; // [m] eind positie
static const float  vMax = 1.0; // [m/s] maximale snelheid
static const float  aMax = 1.0; // [m/s2] maximale acceleratie
static const float  rMax = 0.0; // [m/s3] maximale RUK

static float thetaStart[N_MOTORS];		// Motorpositie aan het begin van de beweging [rad]
static float thetaMax_inc[N_MOTORS];	// Totale incrementele motorverplaatsing [rad]
static float rPiek[N_MOTORS];			// Piek-ruk in motorruimte [rad/s^3]
static float alphaMax[N_MOTORS];		// Piek-hoekacceleratie in motorruimte [rad/s^2]
static float alpha0[N_MOTORS], alpha1[N_MOTORS], alpha2[N_MOTORS], alpha3[N_MOTORS];

static float thetaRef[N_MOTORS];		// Incrementele motorpositie referentie [rad]
static float omegaRef[N_MOTORS];		// Motor-snelheidsreferentie [rad/s]
static float alphaRef[N_MOTORS];		// Motor-acceleratiereferentie [rad/s^2]

static float t0 = 0.0;
static float t1 = 0.0;
static float t2 = 0.0;

static float tau = 0.0; // [s] tijd sinds start van beweging
static float k = 0.0; //
static float dt = 0.0;

Bool Move_ToSetpoint(float x_eindPos, float y_eindPos, float z_eindPos, float Tmax)
{
	mI = 0;

	// Validatie van input parameters
	if (Tmax <= 0.0f)
	{
		return false;
	}

	// Uitlezen van actuele positie
	ReadMotorPositions(motorPos_Rad);

	// Eenmalig nieuw bewegingsprofiel berekenen.
	if(!setupMotionProfileDone)
	{
		t0 = g_time; // starttijd van de beweging

		//float eindPos_M0, eindPos_M1, eindPos_M2 = NULL;
		float tcpTarget[3] = { x_eindPos, y_eindPos, z_eindPos };
	
		// Bepalen van de gewenste motor-eindpositie in motor-radialen.
		if (!DeltaKinematics_Inverse(tcpTarget, motorTargetRad)) // inverse kinematica
		{
			//Indien positie buitenbereik is;
			verplaatsingKlaar = true;
			vPrintString("FOUT: ongeldige eindpositie voor inverse kinematica.\n");
			setupMotionProfileDone = false;
			return true;
		}
		
		// Bepalen van totaal incrementeel te verplaatsen motorhoek per motor
		for (mI = 0; mI < N_MOTORS; mI++)
		{
			// Totaal te verplaatsen motorhoek bepalen.
			thetaStart[mI] = motorPos_Rad[mI];
			
			thetaMax_inc[mI] = motorTargetRad[mI] - motorPos_Rad[mI];
			
			// Bepalen van maximale RUK per motor, zodat binnen Tmax blijft.
			rPiek[mI] = thetaMax_inc[mI] * 32.0/(Tmax*Tmax*Tmax);	// maximale ruk [m/s3]
			alphaMax[mI]	= rPiek[mI]*Tmax/4.0;
		}
		
		t1 = 0.25f * Tmax; // Tmax / 4.0;
		t2 = 0.75f * Tmax; // 3.0 * Tmax / 4.0;

		setupMotionProfileDone = true;
	}
	
	
	// output voltage voor iedere motor bepalen t.o.v. incrementele referentiebewegingsprofiel
	tau = g_time - t0;
	dt = tau - t2;
	for (mI = 0; mI < N_MOTORS; mI++)
	{
		// eenmalig bepalen
		k = alphaMax[mI]/Tmax;

		// opstellen van een referentiebewegingsprofiel voor de regeling
		// 1; Zolang de huidige tijd nog voor het startmoment ligt:
		if (tau <= 0.0f)
		{
			alphaRef[mI]  = 0.0f;
			omegaRef[mI] = 0.0f;
			thetaRef[mI] = 0.0f;

			verplaatsingKlaar = false;
		}
		// 2; Eerste kwart van de beweging: versnelling opbouwen
		else if ( tau <= t1 )		//	2
		{
			alphaRef[mI]	= (4.0f * k) * (tau);			
			omegaRef[mI]	= 2.0f * k * (tau*tau);
			thetaRef[mI]	= (2.0f/3.0f) * k * (tau*tau*tau);
			
			verplaatsingKlaar = false;
		}
		// 3; van versnellen naar vertragen
		else if ( tau <= t2)	//	3
		{
			alphaRef[mI]	=	alphaMax[mI] - 4.0f*k*(tau);

			omegaRef[mI]	=	alphaMax[mI] * (tau) -2.0f * k * ((tau)*(tau)) 
							+ (1.0f/8.0f)*alphaMax[mI]*Tmax;
			
			thetaRef[mI]	=	0.5 * alphaMax[mI] * ((tau)*(tau)) 
							- (2.0f/3.0f) * (k) * ((tau)*(tau)*(tau))
							+ (1.0f/8.0f)*alphaMax[mI]*Tmax*(tau)
							+ (2.0f/(3.0f*64.0f))*alphaMax[mI]*(Tmax*Tmax);
			
			verplaatsingKlaar = false;
		}
		// 4; van vertraging terug naar stilstand, actief remmen tot steeds minder remmen.
		else if ( tau <= Tmax )
		{
			alphaRef[mI]	=	-alphaMax[mI] + 4.0*(k) * (dt);

			omegaRef[mI]	=	-alphaMax[mI] * (dt)
							+ 2.0f*(k) * ((dt)*(dt))
							+ (0.125f)*alphaMax[mI]*Tmax;

			thetaRef[mI]	=	-(alphaMax[mI]/2.0f) * ((dt)*(dt))
							+ (2.0f/3.0f) * (k) * ((dt)*(dt)*(dt))
							+ (0.125f) * alphaMax[mI]*Tmax*(dt)
							+ (22.0f/(192.0f))*alphaMax[mI]*(Tmax*Tmax);

			verplaatsingKlaar = false;
		}
		//	5; op eindpunt en Tmax verstreken.
		else
		{
			alphaRef[mI]	=	0.0f;
			omegaRef[mI]	=	0.0f;
			thetaRef[mI]	=	thetaMax_inc[mI];

			verplaatsingKlaar = true;
			setupMotionProfileDone = false;
		}

		// Bepalen van output voltage mbv Fout, PID_Controller + FeedForward
		Fout_motorRad[mI] = (thetaRef[mI] + thetaStart[mI]) - motorPos_Rad[mI];
		motorControlOutput[mI] = PID_Controller(Fout_motorRad[mI]) + FeedForward(alphaRef[mI]);
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	}
	
	// Daadwerkelijk iedere motor outputvoltage zetten.
	for (mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}
	
	//Tijd bijhouden
	g_time += Ts;

	// Return true als beweging klaar is, anders false.
	return verplaatsingKlaar;

}//END-function
