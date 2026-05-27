/*
 * QuadratureCounters.c
 *
 * Created: 23-11-2023 11:54:44
 *  Author: rasmsmee
 *	Aangepast: Raph van Koeveringe
 */ 

#include "QuadratureCounters.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "vPrintString.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board
// LS7366R encoder counter
#include "QC7366Lib.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "MotorControl.h"

///////////////////////////////////////////////////////////////////////////////
// global
#define ENCODER_CPT						(2048U)
#define QC_MODE							(4U)
#define ENCODER_COUNTS_PER_REVOLUTION	(ENCODER_CPT * QC_MODE) //bij QC4
#define PI								(3.14159f)
#define countsToRad						(- ((2.0f * PI) / (float)ENCODER_COUNTS_PER_REVOLUTION) )


///////////////////////////////////////////////////////////////////////////////
// void QCEncodersSetup(void)
/*
 * Initialiseert alle LS7366R quadraturecounterkanalen voor encoderuitlezing.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; alle counters worden ingesteld, gestart en genuld.
 */
void QCEncodersSetup(void)
{
	uint8_t qcChannel     = 0;
	uint8_t	qcDefaultMode = 0;
	mode_register_t qcModeRegister = QC_MODE_REGISTER_0;
	
	// bitmask, meerdere configuratiebits gecombineerd tot een configuratiewaarde.
	// MODE_QC_4 = vier signalen, stijgende en vallende edges van A en B channel. Dus maximale resolutie.
	// MODE_FREERUNNING = teller blijft doorlopen.
	// INDEX_DISABLE = Z-kanaal wordt niet gebruikt.
	// INDEX_ASYNC = index wordt direct verwerkt, niet gesynchroniseerd. (maar doet niks want disabled).
	// FILTERCLOCK_DIV_2 = digitale input filter, bepaald minimum pulsbreedte. (filter clock = system clock / 2)
	qcDefaultMode = MODE_QC_4 | MODE_FREERUNNING | INDEX_DISABLE | INDEX_ASYNC | FILTERCLOCK_DIV_2;

	// Voor alle beschikbare encoderkanalen:
	for (qcChannel = 0; qcChannel <= QC_MAX_CHANNEL; qcChannel++)
	{
		//mode instellen, counter aanzetten, teller resetten.
		qc_WriteModeRegister(qcChannel, qcModeRegister, qcDefaultMode);
		qc_EnableCounter(qcChannel);
		qc_ClearCountRegister(qcChannel);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void QCEncodersShowCount(const char *idString)
/*
 * Print de ruwe counterwaarde van elk encoderkanaal.
 * Invoer: idString is een tekstlabel voor de debugregel.
 * Uitvoer: geen returnwaarde; schrijft de tellerwaarden naar de debugconsole.
 */
void QCEncodersShowCount(const char *idString)
{
	int32_t qcCountRegister = 0;

	for (uint8_t qcChannel = 0; qcChannel <= QC_MAX_CHANNEL; qcChannel++)
	{
		qcCountRegister = qc_ReadCountRegister(qcChannel);
		vPrintString("%s channel %d: CNT = %8ld\n", idString, qcChannel, (long)qcCountRegister);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void EncodersClearCount(void)
/*
 * Zet alle encoder-counters op nul.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; wist de countregisters van alle motorencoders.
 */
void EncodersClearCount(void)
{
	for (uint8_t qcChannel = 0; qcChannel < N_MOTORS ; qcChannel++) //of QC_MAX_CHANNEL
	{
		qc_ClearCountRegister(MotorQcChannel[qcChannel]);
		removeRegelaarHistory(qcChannel);
		vPrintString("> Encoder count op nul gezet van motor: %d!\n", qcChannel);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void EncoderClearCount(uint8_t qcChannel)
/*
 * Zet een enkele encoder-counter op nul.
 * Invoer: qcChannel kiest de motor/encoder die genuld moet worden.
 * Uitvoer: geen returnwaarde; wist het countregister of print een foutmelding.
 */
void EncoderClearCount(uint8_t qcChannel)
{
	//Check of het binnen motor bereik valt
    if (qcChannel >= N_MOTORS)
    {
	    vPrintString("> ERROR: motorindex buiten bereik!\n");
	    return;
    }
	

	//Zet specifieke QC op nul.
    qc_ClearCountRegister(MotorQcChannel[qcChannel]);
	removeRegelaarHistory(qcChannel);
	vPrintString("> Encoder count op nul gezet van motor: %d!\n", qcChannel);
}

///////////////////////////////////////////////////////////////////////////////
// void LeesMotorPositiesRad(float motorPos_Rad[N_MOTORS])
/*
 * Leest alle motorasposities uit in motor-radialen.
 * Invoer: motorPos_Rad is de outputbuffer met ruimte voor N_MOTORS waarden.
 * Uitvoer: geen returnwaarde; vult motorPos_Rad met de gemeten motorposities.
 */
void LeesMotorPositiesRad(float motorPos_Rad[N_MOTORS])
{
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		motorPos_Rad[mI] = qc_ReadCountRegister(MotorQcChannel[mI]) * countsToRad;
	}
}

///////////////////////////////////////////////////////////////////////////////
// void LeesArmPositiesRad(float armPos_Rad[N_MOTORS])
/*
 * Leest alle armposities uit in arm-radialen.
 * Invoer: armPos_Rad is de outputbuffer met ruimte voor N_MOTORS waarden.
 * Uitvoer: geen returnwaarde; vult armPos_Rad met motorposities omgerekend naar armhoeken.
 */
void LeesArmPositiesRad(float armPos_Rad[N_MOTORS])
{
	for (uint8_t mI = 0; mI < N_MOTORS; mI++)
	{
		armPos_Rad[mI] = (qc_ReadCountRegister(MotorQcChannel[mI]) * countsToRad) / i_twk;
	}
}

///////////////////////////////////////////////////////////////////////////////
// void LeesArmPositieRad(uint8_t motorIndex, float armPos_Rad[N_MOTORS])
/*
 * Leest de armpositie van een enkele motor uit.
 * Invoer: motorIndex kiest de motor, armPos_Rad is de outputbuffer.
 * Uitvoer: geen returnwaarde; schrijft de gemeten armhoek op de index van de motor.
 */
void LeesArmPositieRad(uint8_t motorIndex, float armPos_Rad[N_MOTORS])
{
	//Check of het binnen motor bereik valt
	if (motorIndex >= N_MOTORS)
	{
		vPrintString("> ERROR: motorindex buiten bereik!\n");
		return;
	}
	
	//Bereken Count naar arm-radialen
	armPos_Rad[motorIndex] = ((float)qc_ReadCountRegister(MotorQcChannel[motorIndex]) * countsToRad) / i_twk;
}