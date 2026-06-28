/*
 * Freehand.c
 *
 * Zachte freehand-regeling voor handmatig positioneren.
 */

#include "Freehand.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <math.h>
#include <stdint.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "vPrintString.h"
#include "Map.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "ApplicationTasks.h"
#include "DeltaKinematics.h"
#include "MachinePins.h"
#include "MotorControl.h"
#include "QuadratureCounters.h"
#include "Regelaar.h"

///////////////////////////////////////////////////////////////////////////////
// Freehand tuning
#define FREEHAND_FOLLOW_ALPHA        (0.02f)
#define FREEHAND_KP                  (0.10f)
#define FREEHAND_MAX_VOLTAGE         (0.80f)

#define FREEHAND_GRAVITY_GAIN        (1.5f)
#define FREEHAND_GRAVITY_OFFSET_RAD  (-1.89f)

///////////////////////////////////////////////////////////////////////////////
// file globals
static bool freehandSetupDone = false;
static float motorRef_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};

///////////////////////////////////////////////////////////////////////////////
// void FreeHand_Reset(void)
void FreeHand_Reset(void)
{
	freehandSetupDone = false;
	ZetMotorSpanningen(0.0f);

	for (uint8_t motorIndex = 0U; motorIndex < N_MOTORS; motorIndex++)
	{
		removeRegelaarHistory(motorIndex);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void FreeHand_CompliantControl(void)
/*
 * Voert een zachte P-volgregeling uit.
 *
 * De referentie loopt langzaam mee met de gemeten positie. Daardoor ontstaat bij
 * handmatige beweging een kleine elastische fout in plaats van een harde hold.
 * Gravity-compensatie is als aparte bias toegevoegd en wordt daarna samen met
 * de P-spanning begrensd.
 */
void FreeHand_Control(void)
{
	float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};

	LeesMotorPositiesRad(motorPos_Rad);

	//initialisatie
	if (!freehandSetupDone)
	{
		for (uint8_t motorIndex = 0U; motorIndex < N_MOTORS; motorIndex++)
		{
			motorRef_Rad[motorIndex] = motorPos_Rad[motorIndex];
			removeRegelaarHistory(motorIndex);
		}

		freehandSetupDone = true;
	}

	//Bepalen van volgend-motorkoppel per arm/motor
	for (uint8_t motorIndex = 0U; motorIndex < N_MOTORS; motorIndex++)
	{
		// referentie positie langzaam volgens mee verplaatsen
		motorRef_Rad[motorIndex] += FREEHAND_FOLLOW_ALPHA * (motorPos_Rad[motorIndex] - motorRef_Rad[motorIndex]);

		//Standaard zwaartekracht spanning bepalen
		const float armPos_Rad = motorPos_Rad[motorIndex] / i_twk;
		const float uGravity = FREEHAND_GRAVITY_GAIN * sinf(armPos_Rad + FREEHAND_GRAVITY_OFFSET_RAD);

		//Bij sturende P-regelaar spanning bepalen
		const float error_Rad = motorRef_Rad[motorIndex] - motorPos_Rad[motorIndex];
		const float uP = FREEHAND_KP * error_Rad;
		
		//totale spanning limiteren (systeem "slapper" maken)
		const float uTotal = constrain(uP + uGravity, -FREEHAND_MAX_VOLTAGE, FREEHAND_MAX_VOLTAGE);

		ZetMotorSpanning(motorIndex, uTotal);
	}
}

///////////////////////////////////////////////////////////////////////////////
// bool FreeHand_PrintCurrentTcpPosition(void)
bool FreeHand_PrintCurrentTcpPosition(void)
{
	float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
	float tcp_mm[3] = {0.0f, 0.0f, 0.0f};
	char regel1[DISPLAY_INFO_LINE_LENGTH];
	char regel2[DISPLAY_INFO_LINE_LENGTH];

	LeesMotorPositiesRad(motorPos_Rad);

	//tcp-positie bepalen obv motorposities
	if (!DeltaKinematics_Forward(motorPos_Rad, tcp_mm))
	{
		vPrintString("> FREEHAND: actuele TCP positie ongeldig.\n");
		DisplayInfo_Publish("FreeHand TCP fout", "FK ongeldig");
		return false;
	}

	// XYZpos in terminal
	vPrintString("> FREEHAND TCP: X=%.2f Y=%.2f Z=%.2f mm\n", tcp_mm[0], tcp_mm[1], tcp_mm[2]);

	// regels voor scherm
	snprintf(regel1, sizeof(regel1), "POS: Z%.2f mm", tcp_mm[2]);
	snprintf(regel2, sizeof(regel2), "X%.2f Y%.2f", tcp_mm[0], tcp_mm[1]);
	
	DisplayInfo_Publish(regel1, regel2);

	return true;
}
