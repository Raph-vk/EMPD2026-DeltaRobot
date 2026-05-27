/*
 *  MotionEngine.c
 *  
 *  MotionEngine for controlling the delta robot's motors to follow a desired trajectory.
 *
 *  Created: 10/04/2026
 *  Authors: Raph van Koeveringe
 */
#include "MotionEngine.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <math.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board
#include "DAC4921Lib.h"
#include "QC7366Lib.h"
#include "vPrintString.h"
#include "Map.h" // voor constrain() and fmap()
#include "PortIOLib.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "DeltaKinematics.h"
#include "QuadratureCounters.h"
#include "MotorControl.h"
#include "ControlTask.h"

///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// globals
static const float Ts = 0.001f;      // sample time, must match external clock

static float g_time = 0.0f;          // totale tijd sinds opstarten van de robot in seconden
static float holdTargetPos[N_MOTORS] = {0.0f, 0.0f, 0.0f};

///////////////////////////////////////////////////////////////////////////////
// position control variables
static float motorControlOutput[N_MOTORS];	// Berekende output
static float uDac[N_MOTORS];	// Gelimiteerde uDac output [V] met uDac = Constraint(g,min,max)
	
static float Fout_motorRad[N_MOTORS]			= {0.0f, 0.0f, 0.0f};	// Berekende fout in motor-rad
static float motorPos_Rad[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Gemeten motor-as positie [rad]
static bool setupMotionProfileDone = false;
static bool verplaatsingKlaar = false;
static bool holdSetupDone = false;
static bool gripperSetupDone = false;
static uint16_t currentStep = 0;
static float t0 = 0.0f;
static float t1 = 0.0f;
static float t2 = 0.0f;
static float tau = 0.0f; // [s] tijd sinds start van beweging
static float k = 0.0f;
static float dt = 0.0f;

//static const float degToRad = 0.01745329251994329576923690768489f; //PI / 180.0f; // conversiefactor van graden naar radialen
//static const float RadToDeg = 57.295779513082320876798154814105f; // 180.0f / PI; // conversiefactor van radialen naar graden



///////////////////////////////////////////////////////////////////////////////
// void SequenceRESET(void)
/*
 * Zet de bewegingssequentie terug naar de eerste stap.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; interne sequentie- en setupvlaggen worden gereset.
 */
void SequenceRESET(void)
{
	g_time = 0.0f;
	currentStep = 0;
	setupMotionProfileDone = false;
	verplaatsingKlaar = false;
	holdSetupDone = false;
	gripperSetupDone = false;
}

///////////////////////////////////////////////////////////////////////////////
// bool RunSequence(void)
/*
 * Voert de ingestelde bewegingssequentie stap voor stap uit.
 * Invoer: geen directe parameters; wordt eenmaal per 1 ms control-tick aangeroepen.
 * Uitvoer: true wanneer de volledige sequentie klaar is, anders false.
 */
bool RunSequence(void)
{
	bool stepDone = false;

	switch (currentStep)
	{
		case 0:
		// Move to pick position
		stepDone = Move_ToSetpoint(0.0f, 0.0f, 300.0f, 1.0f);
		break;

		case 1:
		// Close gripper and wait
		stepDone = GripperAtCurrentPosition(true, 0.5f);
		break;

		case 2:
		// Move upward
		stepDone = Move_ToSetpoint(0.0f, 0.0f, 350.0f, 1.0f);
		break;

		case 3:
		// Hold position shortly
		stepDone = HoldCurrentPosition(0.1f);
		break;

		case 4:
		// Open gripper and wait
		stepDone = GripperAtCurrentPosition(false, 1.0f);
		break;

		case 5:
		// Move to next position
		stepDone = Move_ToSetpoint(0.0f, 0.0f, 400.0f, 1.0f);
		break;

		case 6:
		// Sequence finished
		currentStep = 0;
		return true;

		default:
		// Safety fallback
		currentStep = 0;
		return true;
	}

	if (stepDone)
	{
		currentStep++;
	}

	return false;
}




static uint32_t analogPrintCounter = 0;

///////////////////////////////////////////////////////////////////////////////
// static void printAnalogVoltage(float analogvoltage)
/*
 * Print periodiek de gevraagde analoge motorspanning voor debugdoeleinden.
 * Invoer: analogvoltage is de laatst berekende motorspanning in volt.
 * Uitvoer: geen returnwaarde; schrijft alleen naar de debugconsole.
 */
static float spikeVoltage = 0;
static void printAnalogVoltage(float analogvoltage)
{	
	// spike voltage
	if (analogvoltage > spikeVoltage)
	{
		spikeVoltage = analogvoltage;
	}

	// timer
	if (analogPrintCounter >= 1000)
	{
		for (uint8_t i = 0; i < N_MOTORS; i++)
		{
			vPrintString("Motor %u voltage is: %.3f V\n",(unsigned int)i, spikeVoltage);
		}
		analogPrintCounter = 0;
		spikeVoltage = 0;
	}
	else
	{
		analogPrintCounter++;	
	}
}


///////////////////////////////////////////////////////////////////////////////
static const float largeErrorThreshold		= 0.1f;	// Wanneer fout groter is dan 0.1 motor-rad
static const float slowApproachVoltage		= 2.0f;	// [V] Gewenste voltage voor langzame verplaatsing
static float holdMotorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};// Vast te houden motorpositie [rad]

///////////////////////////////////////////////////////////////////////////////
// bool HoldPosition(const float holdArmPos_RAD[N_MOTORS])
/*
 * Regelt alle motorassen naar een vaste armpositie.
 * Invoer: holdArmPos_RAD bevat per arm de gewenste positie in radialen.
 * Uitvoer: true wanneer alle motoren dicht bij de referentie zitten, anders false.
 */
bool HoldPosition(const float holdArmPos_RAD[N_MOTORS])
{
	bool nearReference = true;
	
	// Lees motorposities uit.
	LeesMotorPositiesRad(motorPos_Rad);

	// Voor iedere motor
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
 		// Gewenste armpositie omzetten naar radialen en rekening houden met reducties		
		holdMotorPos_Rad[mI] = i_twk * holdArmPos_RAD[mI];

		// Fout bepalen
		Fout_motorRad[mI] = holdMotorPos_Rad[mI] - motorPos_Rad[mI];

		// Voltage berekenen mbv PID_Controller
		motorControlOutput[mI] = PIDregelaar(mI, Fout_motorRad[mI] );

		printAnalogVoltage(motorControlOutput[mI]); //TEMP

		// Zorg dat waarde niet groter is dan maximale DAC-waarde, en output.
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE); // min/max ongeveer +/-10V.
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}

	return nearReference;
}//End HoldPosition();

///////////////////////////////////////////////////////////////////////////////
// bool HoldCurrentPosition(float waitTime_s)
/*
 * Houdt de actuele armpositie vast voor een opgegeven wachttijd.
 * Invoer: waitTime_s is de minimale vasthoudtijd in seconden.
 * Uitvoer: true wanneer de wachttijd verstreken is, anders false.
 */
bool HoldCurrentPosition(float waitTime_s)
{
	// Als setup nog niet gedaan is,
	if (!holdSetupDone)
	{
		//Bepaal huidige positie
		LeesArmPositiesRad(holdTargetPos);
		
		//check time
		t0 = g_time;
		holdSetupDone = true;
	}

	HoldPosition(holdTargetPos);

	if (g_time - t0 >= waitTime_s)
	{
		holdSetupDone = false;
		g_time += Ts;
		return true; // Hold positie voldaan
	}

	//Tijd bijhouden
	g_time += Ts;
	return false;
}

///////////////////////////////////////////////////////////////////////////////
// bool GripperAtCurrentPosition(bool grab, float waitTime_s)
/*
 * Zet de gripper en houdt de huidige armpositie vast tijdens de wachttijd.
 * Invoer: grab bepaalt sluiten/openen, waitTime_s is de wachttijd in seconden.
 * Uitvoer: true wanneer de wachttijd verstreken is, anders false.
 */
bool GripperAtCurrentPosition(bool grab, float waitTime_s)
{
	//eerste keer
	if (!gripperSetupDone)
	{
		port_SetBit(BIT_GRIPPER, grab);

		//capture currect position
		LeesArmPositiesRad(holdTargetPos);
		
		//check time
		t0 = g_time;
		gripperSetupDone = true;
	}
	
	// Als tijd voorbij is
	else if (g_time - t0 >= waitTime_s)
	{
		gripperSetupDone = false;
		g_time += Ts;
		return true; // Hold positie voldaan
	}

	
	HoldPosition(holdTargetPos);

	//Tijd bijhouden
	g_time += Ts;
	return false;
}

///////////////////////////////////////////////////////////////////////////////
// status bewegingsprofiel
static float motorTargetRad[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Berekende motor-eindpositie [rad]

//static const float  xMax = 0.0; // [m] eind positie
//static const float  vMax = 1.0; // [m/s] maximale snelheid
//static const float  aMax = 1.0; // [m/s2] maximale acceleratie
//static const float  rMax = 0.0; // [m/s3] maximale RUK

static float thetaStart[N_MOTORS];		// Motorpositie aan het begin van de beweging [rad]
static float thetaMax_inc[N_MOTORS];	// Totale incrementele motorverplaatsing [rad]
static float rPiek[N_MOTORS];			// Piek-ruk in motorruimte [rad/s^3]
static float alphaMax[N_MOTORS];		// Piek-hoekacceleratie in motorruimte [rad/s^2]
//static float alpha0[N_MOTORS], alpha1[N_MOTORS], alpha2[N_MOTORS], alpha3[N_MOTORS];

static float thetaRef[N_MOTORS];		// Incrementele motorpositie referentie [rad]
static float omegaRef[N_MOTORS];		// Motor-snelheidsreferentie [rad/s]
static float alphaRef[N_MOTORS];		// Motor-acceleratiereferentie [rad/s^2]

///////////////////////////////////////////////////////////////////////////////
// bool Move_ToSetpoint(float x_mm, float y_mm, float z_mm, float maxTime_s)
/*
 * Beweegt de TCP naar een gewenste positie met een rukbegrensd bewegingsprofiel.
 * Invoer: x_mm, y_mm en z_mm zijn de doelpositie; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool Move_ToSetpoint(float x_mm, float y_mm, float z_mm, float maxTime_s)
{
	// Validatie van input parameters
	if (maxTime_s <= 0.0f)
	{
		return false;
	}

	// Uitlezen van actuele positie
	LeesMotorPositiesRad(motorPos_Rad);

	// Eenmalig nieuw bewegingsprofiel berekenen.
	if(!setupMotionProfileDone)
	{
		t0 = g_time; // starttijd van de beweging

		//float eindPos_M0, eindPos_M1, eindPos_M2 = NULL;
		float tcpTarget[3] = { x_mm, y_mm, z_mm };
	
		// Bepalen van de gewenste motor-eindpositie in motor-radialen.
		if (!DeltaKinematics_Inverse(tcpTarget, motorTargetRad)) // inverse kinematica
		{
			//Indien positie buitenbereik is;
			verplaatsingKlaar = false;
			vPrintString("FOUT: ongeldige eindpositie voor inverse kinematica.\n");
			setupMotionProfileDone = false;
			
			ToState(STATE_PAUSE);
			return false;
		}
		
		// Bepalen van totaal incrementeel te verplaatsen motorhoek per motor
		for (uint8_t mI = 0; mI < N_MOTORS; mI++)
		{
			// Totaal te verplaatsen motorhoek bepalen.
			thetaStart[mI] = motorPos_Rad[mI];
			
			thetaMax_inc[mI] = motorTargetRad[mI] - motorPos_Rad[mI];
			
			// Bepalen van maximale RUK per motor, zodat binnen maxTime_s blijft.
			rPiek[mI] = thetaMax_inc[mI] * 32.0f / (maxTime_s * maxTime_s * maxTime_s);	// maximale ruk [m/s3]
			alphaMax[mI] = rPiek[mI] * maxTime_s / 4.0f;
		}
		
		t1 = 0.25f * maxTime_s; // maxTime_s / 4.0;
		t2 = 0.75f * maxTime_s; // 3.0 * maxTime_s / 4.0;

		setupMotionProfileDone = true;
	}
	
	
	// output voltage voor iedere motor bepalen t.o.v. incrementele referentiebewegingsprofiel
	tau = g_time - t0;
	dt = tau - t2;
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		// eenmalig bepalen
		k = alphaMax[mI] / maxTime_s;

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
							+ (1.0f/8.0f) * alphaMax[mI] * maxTime_s;
			
			thetaRef[mI]	=	0.5 * alphaMax[mI] * ((tau)*(tau)) 
							- (2.0f/3.0f) * (k) * ((tau)*(tau)*(tau))
							+ (1.0f/8.0f) * alphaMax[mI] * maxTime_s * (tau)
							+ (2.0f/(3.0f*64.0f)) * alphaMax[mI] * (maxTime_s * maxTime_s);
			
			verplaatsingKlaar = false;
		}
		// 4; van vertraging terug naar stilstand, actief remmen tot steeds minder remmen.
		else if ( tau <= maxTime_s )
		{
			alphaRef[mI]	=	-alphaMax[mI] + 4.0*(k) * (dt);

			omegaRef[mI]	=	-alphaMax[mI] * (dt)
							+ 2.0f*(k) * ((dt)*(dt))
							+ (0.125f) * alphaMax[mI] * maxTime_s;

			thetaRef[mI]	=	-(alphaMax[mI]/2.0f) * ((dt)*(dt))
							+ (2.0f/3.0f) * (k) * ((dt)*(dt)*(dt))
							+ (0.125f) * alphaMax[mI] * maxTime_s * (dt)
							+ (22.0f/(192.0f)) * alphaMax[mI] * (maxTime_s * maxTime_s);

			verplaatsingKlaar = false;
		}
		//	5; op eindpunt en maxTime_s verstreken.
		else
		{
			alphaRef[mI]	=	0.0f;
			omegaRef[mI]	=	0.0f;
			thetaRef[mI]	=	thetaMax_inc[mI];

			verplaatsingKlaar = true;
			setupMotionProfileDone = false;
			vPrintString("setPointverplaatsing voldaan!\n");
		}

		// Bepalen van output voltage mbv Fout, PID_Controller + FeedForward
		Fout_motorRad[mI] = (thetaRef[mI] + thetaStart[mI]) - motorPos_Rad[mI];
		motorControlOutput[mI] = PIDregelaar(mI, Fout_motorRad[mI]) + FeedForward(alphaRef[mI]);
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	}
	
	// Daadwerkelijk iedere motor outputvoltage zetten.
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}
	
	//Tijd bijhouden
	g_time += Ts;

	// Return true als beweging klaar is, anders false.
	return verplaatsingKlaar;

}//END-function
