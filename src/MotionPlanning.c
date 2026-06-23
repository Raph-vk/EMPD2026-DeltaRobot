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
#include "InputHandlerTask.h"
#include "ApplicationTasks.h"

///////////////////////////////////////////////////////////////////////////////
// const globals
#define DEG_TO_RAD					(0.01745329251994329576923690768489f) // PI / 180.0f

#define Ts							(0.001f)      // sample time, moet gelijk zijn aan externe klok
#define N_TCP_AXES					(3U)

//hop vars
#define HOP_DISTANCE_FACTOR			(0.20f)       // [mm lift / mm XY] gewenste hophoogte uit horizontale afstand
#define HOP_MIN_HEIGHT_MM			(0.0f)        // geen geforceerde minimumhop; gebruik MoveL als hoppen niet nodig is
#define HOP_MAX_HEIGHT_MM			(60.0f)       // absolute begrenzing van extra Z-lift
#define HOP_MAX_Z_ACCEL_MM_S2		(10000.0f)    // effectieve Z-versnellingslimiet voor automatische hophoogte
#define HOP_LIFT_ACCEL_FACTOR		(25.0f)       // factor voor extra parabolische lift

///////////////////////////////////////////////////////////////////////////////
// Interne timers en profieldata
static float g_time = 0.0f;          // totale tijd sinds opstarten van de robot in seconden
static float t0 = 0.0f;              // starttijd van huidige beweging
static float tau = 0.0f;             // [s] tijd sinds start van beweging
static bool verplaatsingKlaar = false;
static bool setupDone = false;

///////////////////////////////////////////////////////////////////////////////
// Profieldata voor MoveJ-bewegingen
static float thetaStart[N_MOTORS] = {0.0f, 0.0f, 0.0f};		// motorpositie bij start [rad]
static float thetaMax_inc[N_MOTORS] = {0.0f, 0.0f, 0.0f};	// totale motorverplaatsing [rad]

///////////////////////////////////////////////////////////////////////////////
// Profieldata voor TCP-bewegingen
static float tcpStart_mm[N_TCP_AXES] = {0.0f, 0.0f, 0.0f};			// TCP-startpositie [mm]
static float tcpTarget_mm[N_TCP_AXES] = {0.0f, 0.0f, 0.0f};		// TCP-eindpositie [mm]
static float tcpMax_inc_mm[N_TCP_AXES] = {0.0f, 0.0f, 0.0f};		// totale TCP-verplaatsing [mm]

//static float motorRef_rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
static float vorigeMotorRef_rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};	// vorige IK-motorreferentie [rad]
static float vorigeMotor_rad_s[N_MOTORS] = {0.0f, 0.0f, 0.0f};	// vorige motor-snelheidsreferentie [rad/s]

///////////////////////////////////////////////////////////////////////////////
// static void printAnalogVoltage(uint8_t m, float analogvoltage)
/*
 * Print periodiek de gevraagde analoge motorspanning voor debugdoeleinden.
 * Invoer: motorindex en de laatst berekende motorspanning in volt.
 * Uitvoer: geen returnwaarde; schrijft alleen naar de debugconsole.
 */
static void printAnalogVoltage(uint8_t m, float analogvoltage)
{
	static uint32_t analogPrintCounter = 0;
	static float spikeVoltage[N_MOTORS] = {0.0f, 0.0f, 0.0f};

	if (fabsf(analogvoltage) > fabsf(spikeVoltage[m]))
	{
		spikeVoltage[m] = analogvoltage;
	}

	if (analogPrintCounter >= 1500) // T_in sec * 3 motoren
	{
		for (uint8_t i = 0; i < N_MOTORS; i++)
		{
			vPrintString("Motor %u voltage is: %.2f V.\n", (unsigned int)i, spikeVoltage[i]);
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
// static bool StopBewegingMetFout(const char *melding)
/*
 * Zet de lopende profielsetup uit en pauzeert de machine bij een ongeldige beweging.
 */
static bool StopBewegingMetFout(const char *melding)
{
	verplaatsingKlaar = false;
	setupDone = false;

	vPrintString("%s\n", melding);
	ToState(STATE_PAUSE);

	return false;
}

///////////////////////////////////////////////////////////////////////////////
// static bool VoegTCPoffsetToe(float tcp_mm[N_TCP_AXES])
/*
 * Voegt de meest recente TCP-offset toe aan X/Y.
 * Uitvoer: true wanneer de offset gewijzigd is, anders false.
 */
static bool VoegTCPoffsetToe(float tcp_mm[N_TCP_AXES])
{
	static OffsetPos_t laatsteOffset = {INFINITY, INFINITY};
	static OffsetPos_t nieuweOffset = {0.0f, 0.0f};
	bool offsetGewijzigd = false;

	// peeken wat de offset waarde is
	if (xQueuePeek(handle_OffsetQueue, &nieuweOffset, 0) == pdTRUE)
	{
		float offsetThreshold = 0.001;
		
		//bepalen of offset-waarde gewijzigd is
		offsetGewijzigd =	fabsf(nieuweOffset.x - laatsteOffset.x) > offsetThreshold ||	fabsf(nieuweOffset.y - laatsteOffset.y) > offsetThreshold;
		laatsteOffset = nieuweOffset;
	}

	tcp_mm[0] += nieuweOffset.x;
	tcp_mm[1] += nieuweOffset.y;

	return offsetGewijzigd;
}


///////////////////////////////////////////////////////////////////////////////
// static void RegelMotoren(...)
/*
 * Rekent voor alle motoren eerst de spanning uit en schrijft daarna alle DACs.
 */
static void RegelMotoren(const float motorRefRad[N_MOTORS], const float EncodedmotorRad[N_MOTORS], const float feedForwardAcc[N_MOTORS])
{
	float uDac[N_MOTORS] = {0.0f, 0.0f, 0.0f};

	// per motor de spanning bepalen
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		const float fout_motorRad = motorRefRad[mI] - EncodedmotorRad[mI];
		float motorControlOutput = PIDregelaar(mI, fout_motorRad);

		if (feedForwardAcc != 0)
		{
			motorControlOutput += FeedForward(feedForwardAcc[mI]);
		}

		printAnalogVoltage(mI, motorControlOutput); // TEMP
		uDac[mI] = constrain(motorControlOutput, DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	}

	// alle motoren tegelijk van spanning voorzien
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		dac_SetOutputVoltage(MotorDacChannel[mI], uDac[mI]);
	}
}

///////////////////////////////////////////////////////////////////////////////
// static void PrintTcpEindFout(...)
/*
 * Print de TCP-eindfout na een MoveL/MoveHop beweging.
 */
static void PrintTcpEindFout(const char *bewegingsNaam, const float motorPos_Rad[N_MOTORS])
{
	float tcpActual_mm[N_TCP_AXES] = {0.0f, 0.0f, 0.0f};

	//bepaald TCP positie en print actuele fout.
	if (DeltaKinematics_Forward(motorPos_Rad, tcpActual_mm))
	{
		const float ex = tcpActual_mm[0] - tcpTarget_mm[0];
		const float ey = tcpActual_mm[1] - tcpTarget_mm[1];
		const float ez = tcpActual_mm[2] - tcpTarget_mm[2];
		const float eAbs = sqrtf(ex*ex + ey*ey + ez*ez);
		vPrintString("%s done. ERROR is: X=%.2f Y=%.2f Z=%.2f mm, abs = %.2f mm\n", bewegingsNaam, ex, ey, ez, eAbs);
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
	t0 = 0.0f;
	tau = 0.0f;
	verplaatsingKlaar = false;
	setupDone = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HOU VAST OP POSITIE FUNCTIES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// bool HoldCurrentPosition(bool grab, float waitTime_s)
/*
 * Zet de gripper en houdt de huidige armpositie vast tijdens de wachttijd.
 * Invoer: grab bepaalt sluiten/openen, waitTime_s is de wachttijd in seconden.
 * Uitvoer: true wanneer de wachttijd verstreken is, anders false.
 */
bool HoldCurrentPosition(bool grab, float waitTime_s)
{
	static float motorHoldRefPos[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	static float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
		
	// grijper uitlezen en gewenste positie vastzetten
	if (!setupDone)
	{
		port_SetBit(BIT_GRIPPER, grab);
		LeesMotorPositiesRad(motorHoldRefPos);
		RegelMotoren(motorHoldRefPos, motorHoldRefPos, 0);
		t0 = g_time;
		setupDone = true;
	}
	// als wachttijd verstreken is.
	else if (g_time - t0 >= waitTime_s)
	{		
		LeesMotorPositiesRad(motorPos_Rad);
		RegelMotoren(motorHoldRefPos, motorPos_Rad, 0);
		setupDone = false;
		g_time += Ts;
		return true;
	}
	// tussentijd positieuitlezen en regelen
	else
	{
		LeesMotorPositiesRad(motorPos_Rad);
		RegelMotoren(motorHoldRefPos, motorPos_Rad, 0);
	}

	g_time += Ts;
	return false;
}

///////////////////////////////////////////////////////////////////////////////
// bool HoldCurrentXYZPosition(bool grab, float waitTime_s)
/*
 * Houdt de huidige TCP-positie vast en compenseert gemeten X/Y-frameverstoring.
 *
 * Bij de eerste aanroep wordt de actuele TCP-positie bepaald uit de gemeten
 * motorposities. Daarna wordt alleen bij een gewijzigde offset een nieuwe
 * gecompenseerde TCP-positie via inverse kinematica omgerekend.
 */
bool HoldCurrentXYZPosition(bool grab, float waitTime_s)
{
	static bool holdTargetGeldig = false;
	static float tcpHoldBasis_mm[N_TCP_AXES] = {0.0f, 0.0f, 0.0f};
	static float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	static float motorRef_rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
		
	// grijper uitlezen en gewenste positie vastzetten
	if (!setupDone)
	{
		port_SetBit(BIT_GRIPPER, grab);
		LeesMotorPositiesRad(motorPos_Rad);

		// tcp positie bepalen tov motorposities
		if (!DeltaKinematics_Forward(motorPos_Rad, tcpHoldBasis_mm))
		{
			vPrintString("FOUT: huidige motorpositie geeft geen geldige TCP-positie.\n");
			ToState(STATE_PAUSE);
			return false;
		}

		verplaatsingKlaar = false;
		t0 = g_time;
		setupDone = true;
		holdTargetGeldig = false;
	}
	//als tijd verstreken is
	else if (g_time - t0 >= waitTime_s)
	{
		setupDone = false;
		verplaatsingKlaar = true;
	}

	float tcpHoldRef_mm[N_TCP_AXES] = {
		tcpHoldBasis_mm[0],
		tcpHoldBasis_mm[1],
		tcpHoldBasis_mm[2]
	};

	//offset aan tcp toevoegen en enkel nieuwe IK uitvoeren indien veranderd is.
	bool offsetGewijzigd = VoegTCPoffsetToe(tcpHoldRef_mm);
	if (offsetGewijzigd || !holdTargetGeldig)
	{
		// bepaal nieuwe motorpositie
		if (!DeltaKinematics_Inverse(tcpHoldRef_mm, motorRef_rad))
		{
			vPrintString("FOUT: Hold XYZ positie ongeldig voor inverse kinematica.\n");
			ToState(STATE_PAUSE);
			return false;
		}

		holdTargetGeldig = true;
	}

	// encoders uitlezen en regelen op positie
	LeesMotorPositiesRad(motorPos_Rad);
	RegelMotoren(motorRef_rad, motorPos_Rad, 0);

	g_time += Ts;
	return verplaatsingKlaar;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JOINTs GECONTROLEERDE BEWEGING
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// bool MoveJ_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
/*
 * Beweegt de TCP naar een gewenste positie met een rukbegrensd motorprofiel.
 * Invoer: x_mm, y_mm en z_mm zijn de doelpositie; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool MoveJ_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
{
	float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float motorRefRad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float feedForwardAcc[N_MOTORS] = {0.0f, 0.0f, 0.0f};

	//controle of het een valide tijd heeft
	if (maxTime_s <= 0.0f)
	{
		return false;
	}

	LeesMotorPositiesRad(motorPos_Rad);
	verplaatsingKlaar = false;

	//in setup incrementele verplaatsing bepalen -> thetaMax_inc[axis]
	if (!setupDone)
	{
		tcpTarget_mm[0] = x_mm;
		tcpTarget_mm[1] = y_mm;
		tcpTarget_mm[2] = z_mm;
		float motorRef_rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};

		vPrintString("start MoveJ X=%.1fmm, Y=%.1fmm, Z=%.1fmm.\n", x_mm, y_mm, z_mm);
		t0 = g_time;

		if (!DeltaKinematics_Inverse(tcpTarget_mm, motorRef_rad))
		{
			return StopBewegingMetFout("FOUT: ongeldige eindpositie voor inverse kinematica.");
		}

		for (uint8_t mI = 0; mI < N_MOTORS; mI++)
		{
			thetaStart[mI] = motorPos_Rad[mI];
			thetaMax_inc[mI] = motorRef_rad[mI] - motorPos_Rad[mI];
		}
		setupDone = true;
	}

	tau = g_time - t0;
	g_time += Ts;
	
	// per motor actuele variabelen in bewegingsprofiel bepalen
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		MotionProfileRef_t ref;
		motionProfile(thetaMax_inc[mI], maxTime_s, tau, &ref);

		verplaatsingKlaar = ref.klaar;
		motorRefRad[mI] = thetaStart[mI] + ref.pos;
		feedForwardAcc[mI] = ref.acc;
	}

	// motoren regelen
	RegelMotoren(motorRefRad, motorPos_Rad, feedForwardAcc);

	if (verplaatsingKlaar)
	{
		setupDone = false;
		PrintTcpEindFout("MoveJ", motorPos_Rad);
	}

	return verplaatsingKlaar;
}

///////////////////////////////////////////////////////////////////////////////
// bool MoveJ_ArmDEG123t(float M1DEG, float M2DEG, float M3DEG, float maxTime_s)
/*
 * Beweegt de armen naar een gewenste hoekpositie met een rukbegrensd motorprofiel.
 * Invoer: M1DEG, M2DEG en M3DEG zijn armhoeken in graden; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool MoveJ_ArmDEG123t(float M1DEG, float M2DEG, float M3DEG, float maxTime_s)
{
	float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float motorRefRad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float feedForwardAcc[N_MOTORS] = {0.0f, 0.0f, 0.0f};

	//controle of het een valide tijd heeft
	if (maxTime_s <= 0.0f)
	{
		return false;
	}

	LeesMotorPositiesRad(motorPos_Rad);
	verplaatsingKlaar = false;

	//in setup incrementele verplaatsing bepalen -> thetaMax_inc[axis]
	if (!setupDone)
	{
		float motorRef_rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};

		vPrintString("start MoveJ M1=%.1fdeg, M2=%.1fdeg, M3=%.1fdeg.\n", M1DEG, M2DEG, M3DEG);
		t0 = g_time;

		// Doelpositie omzetten naar motor-radialen.
		motorRef_rad[0] = -M1DEG * DEG_TO_RAD * i_twk;
		motorRef_rad[1] = -M2DEG * DEG_TO_RAD * i_twk;
		motorRef_rad[2] = -M3DEG * DEG_TO_RAD * i_twk;

		for (uint8_t mI = 0; mI < N_MOTORS; mI++)
		{
			thetaStart[mI] = motorPos_Rad[mI];
			thetaMax_inc[mI] = motorRef_rad[mI] - motorPos_Rad[mI];
		}
		setupDone = true;
	}

	tau = g_time - t0;
	g_time += Ts;

	// per motor actuele variabelen in bewegingsprofiel bepalen
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		MotionProfileRef_t ref;
		motionProfile(thetaMax_inc[mI], maxTime_s, tau, &ref);

		verplaatsingKlaar = ref.klaar;
		motorRefRad[mI] = thetaStart[mI] + ref.pos;
		feedForwardAcc[mI] = ref.acc;
	}

	// motoren regelen
	RegelMotoren(motorRefRad, motorPos_Rad, feedForwardAcc);

	if (verplaatsingKlaar)
	{
		setupDone = false;
		vPrintString("MoveJ_ArmDEG123t voldaan!\n");
	}

	return verplaatsingKlaar;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TCP GECONTROLEERDE BEWEGING
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// static bool InitialiseerTcpBeweging(...)
/*
 * Bepaalt startpunt, eindpunt en totale TCP-verplaatsing voor MoveL.
 */
static bool InitialiseerTcpBeweging(const float motorPos_Rad[N_MOTORS], float x_mm, float y_mm, float z_mm)
{
	// bepalen van of huidige TCP-positie
	if (!DeltaKinematics_Forward(motorPos_Rad, tcpStart_mm))
	{
		return StopBewegingMetFout("FOUT: huidige motorpositie geeft geen geldige TCP-positie.");
	}

	// file-globals voor begin correct invullen
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		vorigeMotorRef_rad[mI] = motorPos_Rad[mI];
		vorigeMotor_rad_s[mI] = 0.0f;
	}

	//TCP target is file-globaal
	tcpTarget_mm[0] = x_mm;
	tcpTarget_mm[1] = y_mm;
	tcpTarget_mm[2] = z_mm;
	
	// incrementeel te verplaatsen waarde bepalen
	for (uint8_t axisI = 0; axisI < N_TCP_AXES; axisI++)
	{
		tcpMax_inc_mm[axisI] = tcpTarget_mm[axisI] - tcpStart_mm[axisI];
	}

	setupDone = true;
	return true;
}

///////////////////////////////////////////////////////////////////////////////
// static float SchatMotorHoekacceleratie(bool bewegingKlaar)
/*
 * Zet een TCP-referentie om naar motorreferenties en regelt de motoren.
 *
 * Maakt gebruik van file-global -> motorRef_rad[mI] en vorigeMotorRef_rad/vorigeMotor_rad_s
 */
static void SchatMotorHoekacceleratie(const float motorRef_rad[N_MOTORS], bool bewegingKlaar, float alphaRefMotors[N_MOTORS] )
{
	// per motor de hoekacceleratie bepalen
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		if (bewegingKlaar)
		{
			alphaRefMotors[mI] = 0.0f;
		}
		else
		{
			//acceleratie berekenen voor feedforward
			float omegaRef = (motorRef_rad[mI] - vorigeMotorRef_rad[mI]) / Ts;
			alphaRefMotors[mI] = (omegaRef - vorigeMotor_rad_s[mI]) / Ts;
			
			//vorige waarde vervangen
			vorigeMotor_rad_s[mI] = omegaRef;
		}
		vorigeMotorRef_rad[mI] = motorRef_rad[mI];
	}
}


///////////////////////////////////////////////////////////////////////////////
// bool MoveL_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
/*
 * Beweegt de TCP in een rechte lijn naar een gewenste positie.
 * Invoer: x_mm, y_mm en z_mm zijn de TCP-doelpositie; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool MoveL_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
{
	float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float motorRef_rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float alphaRefMotors[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float tcpRef_mm[N_TCP_AXES] = {0.0f, 0.0f, 0.0f};

	//controle of het een valide tijd heeft
	if (maxTime_s <= 0.0f)
	{
		vPrintString("maxMoveTime mag niet <=0.0 zijn!\n");
		return false;
	}

	LeesMotorPositiesRad(motorPos_Rad);
	verplaatsingKlaar = false;

	//in setup incrementele verplaatsing bepalen -> tcpMax_inc_mm[axis]
	if (!setupDone)
	{
		vPrintString("start MoveL X=%.1fmm, Y=%.1fmm, Z=%.1fmm.\n", x_mm, y_mm, z_mm);
		t0 = g_time;

		if (!InitialiseerTcpBeweging(motorPos_Rad, x_mm, y_mm, z_mm))
		{
			return false;
		}
	}

	//tijd bijhouden
	tau = g_time - t0;
	g_time += Ts;

	// in &ref de actuele referentie positie,snelheid en acceleratie voor tcp en assen (XYZ) bepalen.
	for (uint8_t axisI = 0; axisI < N_TCP_AXES; axisI++)
	{
		MotionProfileRef_t ref;
		motionProfile(tcpMax_inc_mm[axisI], maxTime_s, tau, &ref);

		verplaatsingKlaar = ref.klaar;
		tcpRef_mm[axisI] = tcpStart_mm[axisI] + ref.pos;
	}

	//Actieve demping toevoegen
	//VoegTCPoffsetToe(tcpRef_mm);

	// motorpositie bij tcpRef punt bepalen (XYZ->M123
	if (!DeltaKinematics_Inverse(tcpRef_mm, motorRef_rad))
	{
		return StopBewegingMetFout("FOUT: MoveL padpunt ongeldig voor inverse kinematica.");
	}

	SchatMotorHoekacceleratie(motorRef_rad, verplaatsingKlaar, alphaRefMotors);
	RegelMotoren(motorRef_rad, motorPos_Rad, alphaRefMotors);
	
	//if (!RegelTcpReferentie(tcpRef_mm, motorPos_Rad, "FOUT: MoveL padpunt ongeldig voor inverse kinematica."))
	//{
	//	return false;
	//}

	if (verplaatsingKlaar)
	{
		setupDone = false;
		PrintTcpEindFout("MoveL", motorPos_Rad);
	}

	return verplaatsingKlaar;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HOP/efficient GECONTROLEERDE BEWEGING
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// static float BerekenHopHoogte(float maxTime_s)
/*
 * Bepaalt een automatische hophoogte uit XY-afstand en beschikbare Z-versnelling.
 */
static float BerekenHopHoogte(float maxTime_s)
{
	// Hop afstand bepalen tov incrementele verplaatsing in XYvlak.
	const float xyDistance_mm = sqrtf(tcpMax_inc_mm[0]*tcpMax_inc_mm[0] + tcpMax_inc_mm[1]*tcpMax_inc_mm[1]);
	const float heightFromDistance_mm = HOP_DISTANCE_FACTOR * xyDistance_mm;
	
	// hop hoogte bij een grote benodigde acceleratie begrenzen
	const float heightFromAccel_mm = (HOP_MAX_Z_ACCEL_MM_S2 * maxTime_s * maxTime_s) / HOP_LIFT_ACCEL_FACTOR;

	// de laagste waarden kiezen.
	float calculatedHeight_mm = heightFromDistance_mm;
	if (heightFromAccel_mm < heightFromDistance_mm)
	{
		calculatedHeight_mm = heightFromAccel_mm;
	}

	return constrain(calculatedHeight_mm, HOP_MIN_HEIGHT_MM, HOP_MAX_HEIGHT_MM);
}

///////////////////////////////////////////////////////////////////////////////
// bool MoveHop_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
/*
 * Beweegt de TCP via een automatisch geschaalde parabolische hop naar de doelpositie.
 * Invoer: x_mm, y_mm en z_mm zijn de TCP-doelpositie; maxTime_s is de bewegingstijd.
 * Uitvoer: true wanneer de beweging klaar is, anders false.
 */
bool MoveHop_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s)
{
	static float hopHeight_mm = 0.0f;	// berekende extra Z-lift [mm]
	float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};	//gemeten motorpositie
	float motorRef_rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};	//referentie motorpostie
	float alphaRefMotors[N_MOTORS] = {0.0f, 0.0f, 0.0f};//referentie motor hoekacceleratie (voor feedforward)
	float tcpRef_mm[N_TCP_AXES] = {0.0f, 0.0f, 0.0f};	// XYZ referentie

	// oneindig snel mag/kan niet
	if (maxTime_s <= 0.0f)
	{
		vPrintString("maxMoveTime kan niet nul zijn!\n");
		return false;
	}

	LeesMotorPositiesRad(motorPos_Rad);
	verplaatsingKlaar = false;

	// bewegingsprofiel bepalen
	if (!setupDone)
	{
		vPrintString("start MoveHop X=%.1fmm, Y=%.1fmm, Z=%.1fmm.\n", x_mm, y_mm, z_mm);
		t0 = g_time;

		if (!InitialiseerTcpBeweging(motorPos_Rad, x_mm, y_mm, z_mm))
		{
			return false;
		}

		//berekenen van hopHoogte
		hopHeight_mm = BerekenHopHoogte(maxTime_s);
		vPrintString("MoveHop hoogte: %.2f mm\n", hopHeight_mm);
	}

	//tijd bijhouden
	tau = g_time - t0;
	g_time += Ts;

	// Referentiepositie bepalen via een genormaliseerde padvoortgang.
	// motionProfile(1.0f, ...) geeft geen echte mm-verplaatsing terug,
	// maar een factor s van 0.0 tot 1.0 over de volledige bewegingstijd.
	MotionProfileRef_t pathRef;
	motionProfile(1.0f, maxTime_s, tau, &pathRef);
	verplaatsingKlaar = pathRef.klaar;

	// s is de procentuele voortgang van het pad per tick:
	const float s = pathRef.pos;
	
	// Extra Z-lift voor de hop.
	// De term 4*s*(1-s) vormt een parabool:
	const float hopLift_mm = hopHeight_mm * 4.0f * s * (1.0f - s); // - bij s = 0.0 is de lift 0, bij s = 0.5 is de lift maximaal, bij s = 1.0 is de lift weer 0

	// X, Y en Z schalen in zijn eigen verplaatsing procentueel mee met padvoortgang s.
	tcpRef_mm[0] = tcpStart_mm[0] + tcpMax_inc_mm[0] * s;
	tcpRef_mm[1] = tcpStart_mm[1] + tcpMax_inc_mm[1] * s;
	tcpRef_mm[2] = tcpStart_mm[2] + tcpMax_inc_mm[2] * s + hopLift_mm; //additioneel de hopLift

	//Actieve demping toevoegen
	//VoegTCPoffsetToe(tcpRef_mm);

	// motorpositie bij tcpRef punt bepalen (XYZ->M123
	if (!DeltaKinematics_Inverse(tcpRef_mm, motorRef_rad))
	{
		return StopBewegingMetFout("FOUT: MoveL padpunt ongeldig voor inverse kinematica.");
	}

	//aansturen van de motoren
	SchatMotorHoekacceleratie(motorRef_rad, verplaatsingKlaar, alphaRefMotors);
	RegelMotoren(motorRef_rad, motorPos_Rad, alphaRefMotors);

	//als 
	if (verplaatsingKlaar)
	{
		setupDone = false;
		PrintTcpEindFout("MoveHop", motorPos_Rad);
	}

	return verplaatsingKlaar;
}
