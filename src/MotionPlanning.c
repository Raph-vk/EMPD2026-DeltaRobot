
/*
 * MotionPlanning.c
 *
 * Created: 03/06/2026 15:08:43
 *  Author: raphv
 */ 

#include "MotionPlanning.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <math.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board
#include "DAC4921Lib.h"
#include "vPrintString.h"
#include "Map.h" // voor constrain() and fmap()
#include "PortIOLib.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "DeltaKinematics.h"
#include "QuadratureCounters.h"
#include "MotorControl.h"
#include "ControlTask.h"
#include "MotionProfiles.h"
#include "Regelaar.h"

///////////////////////////////////////////////////////////////////////////////
// const globals
#define Ts							(0.001f)      // sample time, must match external clock
#define N_TCP_AXES					(3U)
#define DEG_TO_RAD					(0.01745329251994329576923690768489f) // PI / 180.0f

///////////////////////////////////////////////////////////////////////////////
// globals vars
static float g_time = 0.0f;          // totale tijd sinds opstarten van de robot in seconden
static float holdTargetPos[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	
///////////////////////////////////////////////////////////////////////////////
// position control global variables
static float motorControlOutput[N_MOTORS];	// Berekende output
static float uDac[N_MOTORS];	// Gelimiteerde uDac output [V] met uDac = Constraint(g,min,max)
	
static float Fout_motorRad[N_MOTORS] = {0.0f, 0.0f, 0.0f};	// Berekende fout in motor-rad
static float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};	// Gemeten motor-as positie [rad]
static bool setupMotionProfileDone = false;
static bool verplaatsingKlaar = false;
static bool gripperSetupDone = false;
static float t0 = 0.0f;
static float tau = 0.0f; // [s] tijd sinds start van beweging

///////////////////////////////////////////////////////////////////////////////
// static void printAnalogVoltage(float analogvoltage)
/*
 * Print periodiek de gevraagde analoge motorspanning voor debugdoeleinden.
 * Invoer: analogvoltage is de laatst berekende motorspanning in volt.
 * Uitvoer: geen returnwaarde; schrijft alleen naar de debugconsole.
 */
static uint32_t analogPrintCounter = 0;
static float spikeVoltage[N_MOTORS] = {0.0f, 0.0f, 0.0f};
static void printAnalogVoltage(uint8_t m,  float analogvoltage)
{
	// spike voltage
	if (fabsf(analogvoltage) > fabsf(spikeVoltage[m]))
	{
		spikeVoltage[m] = analogvoltage;
	}

	// timer
	if (analogPrintCounter >= 3000)
	{
		for (uint8_t i = 0; i < N_MOTORS; i++)
		{
			//vPrintString("Motor %u voltage is: %.2f V\n",(unsigned int)i, spikeVoltage[i]);
			spikeVoltage[i] = 0.0f;
		}
		analogPrintCounter = 0;
	}
	else
	{
		analogPrintCounter++;	
	}
}

///////////////////////////////////////////////////////////////////////////////
// void MotionPlanning_RESET(void)
/*
 * Zet de interne bewegingsprofielen en wachttimers terug naar hun begintoestand.
 */
void MotionPlanning_RESET(void)
{
	g_time = 0.0f;
	setupMotionProfileDone = false;
	verplaatsingKlaar = false;
	gripperSetupDone = false;
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
static void HoldPosition(const float holdArmPos_RAD[N_MOTORS])
{	
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
		
		printAnalogVoltage(mI, motorControlOutput[mI]); //TEMP

		// Zorg dat waarde niet groter is dan maximale DAC-waarde, en output.
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE); // min/max ongeveer +/-10V.
		//dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}
	
	//Iedere motor tegelijk bijregelen.
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}
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

static float thetaStart[N_MOTORS];		// Motorpositie aan het begin van de beweging [rad]
static float thetaMax_inc[N_MOTORS];	// Totale incrementele motorverplaatsing [rad]
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
	verplaatsingKlaar = false;
	
	// Eenmalig nieuw bewegingsprofiel berekenen.
	if(!setupMotionProfileDone)
	{
		vPrintString("> instellen beweging.\n");
		t0 = g_time; // starttijd van de beweging

		//float eindPos_M0, eindPos_M1, eindPos_M2 = NULL;
		float tcpTarget[3] = { x_mm, y_mm, z_mm };
	
		// Bepalen van de gewenste motor-eindpositie in motor-radialen.
		if (!DeltaKinematics_Inverse(tcpTarget, motorTargetRad)) // inverse kinematica
		{	
			//Indien positie buitenbereik is;
			vPrintString("FOUT: ongeldige eindpositie voor inverse kinematica.\n");
			verplaatsingKlaar = false;
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
		}

		setupMotionProfileDone = true;
	}//end setup

	
	// output voltage voor iedere motor bepalen t.o.v. incrementele referentiebewegingsprofiel
	tau = g_time - t0;
	//Tijd bijhouden
	g_time += Ts;
	
	
	// Per motor beweging bepalen
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		//Bewegingsprofiel bepalen
		MotionProfileRef_t ref;
		motionProfile(thetaMax_inc[mI], maxTime_s, tau, &ref);
		
		vPrintString("pos: %.1f. velocity: %.1f. acceleration: %.1f.", ref.pos, ref.vel, ref.acc);

		verplaatsingKlaar = ref.klaar;


		// Bepalen van output voltage mbv Fout, PID_Controller + FeedForward
		Fout_motorRad[mI] = (ref.pos + thetaStart[mI]) - motorPos_Rad[mI];
		motorControlOutput[mI] = PIDregelaar(mI, Fout_motorRad[mI]) + FeedForward(ref.acc);
		printAnalogVoltage(mI, motorControlOutput[mI]); //TEMP
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	}
	
	// Daadwerkelijk iedere motor outputvoltage zetten. (TEGELIJK REGELEN)
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}
	
	//Als verplaatsing voldaan is
	if(verplaatsingKlaar)
	{
		setupMotionProfileDone = false;
		vPrintString("MoveJ_XYZt voldaan!\n");
	}

	// Return true als beweging klaar is, anders false.
	return verplaatsingKlaar;

}//END-function

///////////////////////////////////////////////////////////////////////////////
// bool MoveJ_ArmDEG123t(const float armRadians[N_MOTORS], float maxTime_s)
/*
 * Beweegt de armen naar een gewenste hoekpositie met een rukbegrensd bewegingsprofiel.
 * Invoer: armRadians bevat de gewenste armhoeken in radialen; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool MoveJ_ArmDEG123t(float M1DEG, float M2DEG, float M3DEG, float maxTime_s)
{
	// Validatie van input parameters
	if (maxTime_s <= 0.0f)
	{
		return false;
	}

	// Uitlezen van actuele positie
	LeesMotorPositiesRad(motorPos_Rad);
	verplaatsingKlaar = false;

	// Eenmalig nieuw bewegingsprofiel berekenen.
	if(!setupMotionProfileDone)
	{
		t0 = g_time; // starttijd van de beweging

		// Doelpositie direct overnemen in motor-radialen.
		// Geen inverse kinematica meer.
		motorTargetRad[0] = -M1DEG * DEG_TO_RAD * i_twk;
		motorTargetRad[1] = -M2DEG * DEG_TO_RAD * i_twk;
		motorTargetRad[2] = -M3DEG * DEG_TO_RAD * i_twk;
		
		// Bepalen van totaal incrementeel te verplaatsen motorhoek per motor
		for (uint8_t mI = 0; mI < N_MOTORS; mI++)
		{
			// Totaal te verplaatsen motorhoek bepalen.
			thetaStart[mI] = motorPos_Rad[mI];
			thetaMax_inc[mI] = motorTargetRad[mI] - motorPos_Rad[mI];
		}

		setupMotionProfileDone = true;
	}
	
	
	//tijd bepalen in bewegingprofiel (per tick)
	tau = g_time - t0;
	//Tijd bijhouden
	g_time += Ts;
	
	// output voltage voor iedere motor bepalen t.o.v. incrementele referentiebewegingsprofiel
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		MotionProfileRef_t ref;
		motionProfile( thetaMax_inc[mI], maxTime_s, tau, &ref);
		verplaatsingKlaar = ref.klaar;

		// Bepalen van output voltage mbv Fout, PID_Controller + FeedForward
		Fout_motorRad[mI] = (ref.pos + thetaStart[mI]) - motorPos_Rad[mI];
		motorControlOutput[mI] = PIDregelaar(mI, Fout_motorRad[mI]) + FeedForward(ref.acc);
		printAnalogVoltage(mI, motorControlOutput[mI]); //TEMP
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	}
	
	// Daadwerkelijk iedere motor outputvoltage zetten.
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}

	if(verplaatsingKlaar)
	{
		setupMotionProfileDone = false;
		vPrintString("MoveJ_ArmDEG123t voldaan!\n");
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

static float tcpRef_mm[N_TCP_AXES]			= {0.0f, 0.0f, 0.0f};	// TCP-positiereferentie [mm]

static float motorTargetPrevRad[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Vorige IK-motorreferentie [rad]
static float motorOmegaPrevRad_s[N_MOTORS]	= {0.0f, 0.0f, 0.0f};	// Vorige motor-snelheidsreferentie [rad/s]
	
static float omegaRef[N_MOTORS];		// Motor-snelheidsreferentie [rad/s]
static float alphaRef[N_MOTORS];		// Motor-acceleratiereferentie [rad/s^2]
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
	verplaatsingKlaar = false;

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
		}

		setupMotionProfileDone = true;
		
	}//eind-setupMotionProfile



	
	//Tijd bepalen in bewegingprofiel (per tick)
	tau = g_time - t0;
	//Tijd bijhouden
	g_time += Ts; 

	// TCP-gewenste/referentie positie bepalen
	for (uint8_t axisI = 0; axisI < N_TCP_AXES; axisI++)
	{
		MotionProfileRef_t ref;
		motionProfile(tcpMax_inc_mm[axisI], maxTime_s, tau, &ref);
		verplaatsingKlaar = ref.klaar;
		tcpRef_mm[axisI] = tcpStart_mm[axisI] + ref.pos;
	}//end-AxisForloop


	// de TCP-referentie omzetten naar motorreferenties.
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
		printAnalogVoltage(mI, motorControlOutput[mI]); //TEMP
		uDac[mI] = constrain(motorControlOutput[mI], DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	}

	// Daadwerkelijk iedere motor outputvoltage zetten.
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
		
		motorTargetPrevRad[mI] = motorTargetRad[mI];
		motorOmegaPrevRad_s[mI] = omegaRef[mI];
	}

	if(verplaatsingKlaar)
	{
		setupMotionProfileDone = false;
		float tcpActual_mm[3];

		if (DeltaKinematics_Forward(motorPos_Rad, tcpActual_mm))
		{
			float ex = tcpActual_mm[0] - tcpTarget_mm[0];
			float ey = tcpActual_mm[1] - tcpTarget_mm[1];
			float ez = tcpActual_mm[2] - tcpTarget_mm[2];
			float eAbs = sqrtf(ex*ex + ey*ey + ez*ez);

			vPrintString("MoveL done. ERROR is: X=%.2f Y=%.2f Z=%.2f mm, abs = %.2f mm\n",
			ex, ey, ez, eAbs);
		}
	}
	// Return true als beweging klaar is, anders false.
	return verplaatsingKlaar;

}//END-function
