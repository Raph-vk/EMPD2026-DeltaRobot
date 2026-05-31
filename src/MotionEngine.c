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
// const globals
#define Ts							(0.001f)      // sample time, must match external clock
#define N_TCP_AXES		(3U)
//#define DEG_TO_RAD					(0.01745329251994329576923690768489f) //PI / 180.0f; // conversiefactor van graden naar radialen
//#define const float RAD_TO_DEG		(57.295779513082320876798154814105f) // 180.0f / PI; // conversiefactor van radialen naar graden

///////////////////////////////////////////////////////////////////////////////
// globals vars
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


///////////////////////////////////////////////////////////////////////////////
// static void printAnalogVoltage(float analogvoltage)
/*
 * Print periodiek de gevraagde analoge motorspanning voor debugdoeleinden.
 * Invoer: analogvoltage is de laatst berekende motorspanning in volt.
 * Uitvoer: geen returnwaarde; schrijft alleen naar de debugconsole.
 */
static uint32_t analogPrintCounter = 0;
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
			//vPrintString("Motor %u voltage is: %.3f V\n",(unsigned int)i, spikeVoltage);
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
		stepDone = HoldCurrentPosition(true, 0.1f);
		break;

		case 1:
		// Close gripper and wait
		stepDone = MoveJ_XYZt(-75.0f, 75.0f, -490.0f, 1.0f);
		break;

		case 2:
		// Move upward
		stepDone = MoveJ_XYZt(-75.0f, -75.0f, -490.0f, 1.0f);
		break;

		case 3:
		// Hold position shortly
		stepDone = MoveJ_XYZt(75.0f, -75.0f, -490.0f, 1.0f);
		break;

		case 4:
		// Hold position shortly
		stepDone = MoveJ_XYZt(75.0f, 75.0f, -490.0f, 1.0f);
		break;

		case 5:
		// Open gripper and wait490
		stepDone = HoldCurrentPosition(false, 1.0f);
		break;

		case 6:
		// Move to next position
		stepDone = MoveJ_ArmRAD123t(0.0f, 0.0f, 0.0f, 5.0f);
		break;

		case 7:
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

///////////////////////////////////////////////////////////////////////////////
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
// bool HoldCurrentPosition(bool grab, float waitTime_s)
/*
 * Zet de gripper en houdt de huidige armpositie vast tijdens de wachttijd.
 * Invoer: grab bepaalt sluiten/openen, waitTime_s is de wachttijd in seconden.
 * Uitvoer: true wanneer de wachttijd verstreken is, anders false.
 */
bool HoldCurrentPosition(bool grab, float waitTime_s)
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
// bool MoveJ_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
/*
 * Beweegt de TCP naar een gewenste positie met een rukbegrensd bewegingsprofiel.
 * Invoer: x_mm, y_mm en z_mm zijn de doelpositie; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool MoveJ_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
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

	if(verplaatsingKlaar)
	{
		vPrintString("setPointverplaatsing voldaan!\n");
	}

	// Return true als beweging klaar is, anders false.
	return verplaatsingKlaar;

}//END-function

///////////////////////////////////////////////////////////////////////////////
// bool MoveJ_ArmRAD123t(const float armRadians[N_MOTORS], float maxTime_s)
/*
 * Beweegt de armen naar een gewenste hoekpositie met een rukbegrensd bewegingsprofiel.
 * Invoer: armRadians bevat de gewenste armhoeken in radialen; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool MoveJ_ArmRAD123t(float M1RAD, float M2RAD, float M3RAD, float maxTime_s)
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

		// Doelpositie direct overnemen in motor-radialen.
		// Geen inverse kinematica meer.
		motorTargetRad[0] = M1RAD;
		motorTargetRad[1] = M2RAD;
		motorTargetRad[2] = M3RAD;
		
		// Bepalen van totaal incrementeel te verplaatsen motorhoek per motor
		for (uint8_t mI = 0; mI < N_MOTORS; mI++)
		{
			// Totaal te verplaatsen motorhoek bepalen.
			thetaStart[mI] = motorPos_Rad[mI];
			
			thetaMax_inc[mI] = motorTargetRad[mI] - motorPos_Rad[mI];
			
			// Bepalen van maximale RUK per motor, zodat binnen maxTime_s blijft.
			rPiek[mI] = thetaMax_inc[mI] * 32.0f / (maxTime_s * maxTime_s * maxTime_s);
			alphaMax[mI] = rPiek[mI] * maxTime_s / 4.0f;
		}
		
		t1 = 0.25f * maxTime_s;
		t2 = 0.75f * maxTime_s;

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
		else if ( tau <= t1 )
		{
			alphaRef[mI]	= (4.0f * k) * (tau);			
			omegaRef[mI]	= 2.0f * k * (tau*tau);
			thetaRef[mI]	= (2.0f/3.0f) * k * (tau*tau*tau);
			
			verplaatsingKlaar = false;
		}
		// 3; van versnellen naar vertragen
		else if ( tau <= t2)
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
		// 5; op eindpunt en maxTime_s verstreken.
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

	if(verplaatsingKlaar)
	{
		vPrintString("setPointverplaatsing voldaan!\n");
	}

	// Return true als beweging klaar is, anders false.
	return verplaatsingKlaar;

}//END-function




///////////////////////////////////////////////////////////////////////////////
// bool MoveL_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
/*
 * Beweegt de TCP in een rechte lijn naar een gewenste positie.
 * Invoer: x_mm, y_mm en z_mm zijn de TCP-doelpositie; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
static float tcpStart_mm[N_TCP_AXES]		= {0.0f, 0.0f, 0.0f};	// TCP-startpositie [mm]
static float tcpTarget_mm[N_TCP_AXES]		= {0.0f, 0.0f, 0.0f};	// TCP-eindpositie [mm]
static float tcpMax_inc_mm[N_TCP_AXES]		= {0.0f, 0.0f, 0.0f};	// Totale incrementele TCP-verplaatsing [mm]
static float tcpRPiek_mm_s3[N_TCP_AXES]		= {0.0f, 0.0f, 0.0f};	// Piek-ruk in TCP-ruimte [mm/s^3]
static float tcpAccMax_mm_s2[N_TCP_AXES]	= {0.0f, 0.0f, 0.0f};	// Piek-acceleratie in TCP-ruimte [mm/s^2]

static float tcpRef_mm[N_TCP_AXES]			= {0.0f, 0.0f, 0.0f};	// TCP-positiereferentie [mm]
static float tcpRef_inc_mm[N_TCP_AXES]		= {0.0f, 0.0f, 0.0f};	// Incrementele TCP-positiereferentie [mm]
static float tcpVelRef_mm_s[N_TCP_AXES]		= {0.0f, 0.0f, 0.0f};	// TCP-snelheidsreferentie [mm/s]
static float tcpAccRef_mm_s2[N_TCP_AXES]	= {0.0f, 0.0f, 0.0f};	// TCP-acceleratiereferentie [mm/s^2]

static float motorTargetPrevRad[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Vorige IK-motorreferentie [rad]
static float motorOmegaPrevRad_s[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Vorige motor-snelheidsreferentie [rad/s]
///////////////////////////////////////////////////////////////////////////////	
bool MoveL_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
{
	// Validatie van input parameters
	if (maxTime_s <= 0.0f)
	{
		vPrintString("maxMoveTime problem!\n");
		return false;
	}

	// Uitlezen van actuele positie
	LeesMotorPositiesRad(motorPos_Rad);

	// Eenmalig nieuw TCP-bewegingsprofiel berekenen.
	if(!setupMotionProfileDone)
	{
		t0 = g_time; // starttijd van de beweging

		// Huidige TCP-positie bepalen uit de actuele motorhoeken.
		if (!DeltaKinematics_Forward(motorPos_Rad, tcpStart_mm))
		{
			verplaatsingKlaar = false;
			vPrintString("FOUT: huidige motorpositie geeft geen geldige TCP-positie.\n");
			setupMotionProfileDone = false;

			ToState(STATE_PAUSE);
			return false;
		}

		// Startpunt vooraf controleren met inverse kinematica.
		if (!DeltaKinematics_Inverse(tcpStart_mm, motorTargetRad))
		{
			verplaatsingKlaar = false;
			vPrintString("FOUT: huidige TCP-positie ongeldig voor inverse kinematica.\n");
			setupMotionProfileDone = false;

			ToState(STATE_PAUSE);
			return false;
		}

		//
		for (uint8_t mI = 0; mI < N_MOTORS; mI++)
		{
			motorTargetPrevRad[mI] = motorTargetRad[mI];
			motorOmegaPrevRad_s[mI] = 0.0f;
		}

		//ingegeven posities in array voegen.
		tcpTarget_mm[0] = x_mm;
		tcpTarget_mm[1] = y_mm;
		tcpTarget_mm[2] = z_mm;
		
		//Eindbepalen en controleren of geldig is.
		if (!DeltaKinematics_Inverse(tcpTarget_mm, motorTargetRad))
		{
			verplaatsingKlaar = false;
			vPrintString("FOUT: ongeldige eindpositie voor inverse kinematica.\n");
			setupMotionProfileDone = false;

			ToState(STATE_PAUSE);
			return false;
		}

		// Bepalen van totaal incrementeel te verplaatsen TCP-afstand per as.
		for (uint8_t axisI = 0; axisI < N_TCP_AXES; axisI++)
		{
			tcpMax_inc_mm[axisI] = tcpTarget_mm[axisI] - tcpStart_mm[axisI];

			// Zelfde rukbegrensde profiel als MoveJ, maar nu in TCP-ruimte.
			tcpRPiek_mm_s3[axisI] = tcpMax_inc_mm[axisI] * 32.0f / (maxTime_s * maxTime_s * maxTime_s);
			tcpAccMax_mm_s2[axisI] = tcpRPiek_mm_s3[axisI] * maxTime_s / 4.0f;
		}

		t1 = 0.25f * maxTime_s;
		t2 = 0.75f * maxTime_s;

		setupMotionProfileDone = true;
		
	}//eind-setupMotionProfile

	// TCP-referentiepositie bepalen t.o.v. incrementele referentiebewegingsprofiel
	tau = g_time - t0;
	dt = tau - t2;
	for (uint8_t axisI = 0; axisI < N_TCP_AXES; axisI++)
	{
		// eenmalig bepalen
		k = tcpAccMax_mm_s2[axisI] / maxTime_s;

		// opstellen van een TCP-referentiebewegingsprofiel voor de regeling
		// 1; Zolang de huidige tijd nog voor het startmoment ligt:
		if (tau <= 0.0f)
		{
			tcpAccRef_mm_s2[axisI] = 0.0f;
			tcpVelRef_mm_s[axisI] = 0.0f;
			tcpRef_inc_mm[axisI] = 0.0f;

			verplaatsingKlaar = false;
		}
		// 2; Eerste kwart van de beweging: versnelling opbouwen
		else if ( tau <= t1 )
		{
			tcpAccRef_mm_s2[axisI] = (4.0f * k) * (tau);
			tcpVelRef_mm_s[axisI] = 2.0f * k * (tau*tau);
			tcpRef_inc_mm[axisI] = (2.0f/3.0f) * k * (tau*tau*tau);

			verplaatsingKlaar = false;
		}
		// 3; van versnellen naar vertragen
		else if ( tau <= t2)
		{
			tcpAccRef_mm_s2[axisI] = tcpAccMax_mm_s2[axisI] - 4.0f*k*(tau);

			tcpVelRef_mm_s[axisI] = tcpAccMax_mm_s2[axisI] * (tau) -2.0f * k * ((tau)*(tau))
								+ (1.0f/8.0f) * tcpAccMax_mm_s2[axisI] * maxTime_s;

			tcpRef_inc_mm[axisI] = 0.5f * tcpAccMax_mm_s2[axisI] * ((tau)*(tau))
								- (2.0f/3.0f) * (k) * ((tau)*(tau)*(tau))
								+ (1.0f/8.0f) * tcpAccMax_mm_s2[axisI] * maxTime_s * (tau)
								+ (2.0f/(3.0f*64.0f)) * tcpAccMax_mm_s2[axisI] * (maxTime_s * maxTime_s);

			verplaatsingKlaar = false;
		}
		// 4; van vertraging terug naar stilstand, actief remmen tot steeds minder remmen.
		else if ( tau <= maxTime_s )
		{
			tcpAccRef_mm_s2[axisI] = -tcpAccMax_mm_s2[axisI] + 4.0f*(k) * (dt);

			tcpVelRef_mm_s[axisI] = -tcpAccMax_mm_s2[axisI] * (dt)
								+ 2.0f*(k) * ((dt)*(dt))
								+ (0.125f) * tcpAccMax_mm_s2[axisI] * maxTime_s;

			tcpRef_inc_mm[axisI] = -(tcpAccMax_mm_s2[axisI]/2.0f) * ((dt)*(dt))
								+ (2.0f/3.0f) * (k) * ((dt)*(dt)*(dt))
								+ (0.125f) * tcpAccMax_mm_s2[axisI] * maxTime_s * (dt)
								+ (22.0f/(192.0f)) * tcpAccMax_mm_s2[axisI] * (maxTime_s * maxTime_s);

			verplaatsingKlaar = false;
		}
		// 5; op eindpunt en maxTime_s verstreken.
		else
		{
			tcpAccRef_mm_s2[axisI] = 0.0f;
			tcpVelRef_mm_s[axisI] = 0.0f;
			tcpRef_inc_mm[axisI] = tcpMax_inc_mm[axisI];

			verplaatsingKlaar = true;
			setupMotionProfileDone = false;
		}

		tcpRef_mm[axisI] = tcpStart_mm[axisI] + tcpRef_inc_mm[axisI];
	}//end-AxisForloop

	// Iedere clock tick de TCP-referentie omzetten naar motorreferenties.
	if (!DeltaKinematics_Inverse(tcpRef_mm, motorTargetRad))
	{
		verplaatsingKlaar = false;
		vPrintString("FOUT: MoveL padpunt ongeldig voor inverse kinematica.\n");
		setupMotionProfileDone = false;

		ToState(STATE_PAUSE);
		return false;
	}

	// Bepalen van output voltage mbv Fout, PID_Controller + FeedForward
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		// Het profiel is in TCP-ruimte; motor-acceleratie wordt uit opeenvolgende IK-referenties geschat.
		if (verplaatsingKlaar)
		{
			omegaRef[mI] = 0.0f;
			alphaRef[mI] = 0.0f;
		}
		else
		{
			omegaRef[mI] = (motorTargetRad[mI] - motorTargetPrevRad[mI]) / Ts;
			alphaRef[mI] = (omegaRef[mI] - motorOmegaPrevRad_s[mI]) / Ts;
		}

		Fout_motorRad[mI] = motorTargetRad[mI] - motorPos_Rad[mI];
		motorControlOutput[mI] = PIDregelaar(mI, Fout_motorRad[mI]) + FeedForward(alphaRef[mI]);
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	}

	// Daadwerkelijk iedere motor outputvoltage zetten.
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
		motorTargetPrevRad[mI] = motorTargetRad[mI];
		motorOmegaPrevRad_s[mI] = omegaRef[mI];
	}

	//Tijd bijhouden
	g_time += Ts;

	if(verplaatsingKlaar)
	{
		vPrintString("MoveL verplaatsing voldaan!\n");
	}

	// Return true als beweging klaar is, anders false.
	return verplaatsingKlaar;

}//END-function