/*
 * QuadratureCounters.c
 *
 * Created: 23-11-2023 11:54:44
 *  Author: rasmsmee
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "CommandConsole.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board
// LS7366R encoder counter
#include "QC7366Lib.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "QuadratureCounters.h"

///////////////////////////////////////////////////////////////////////////////
// void QCEncodersSetup(void)
// 
void QCEncodersSetup(void)
{
	uint8_t qcChannel     = 0;
	uint8_t	qcDefaultMode = 0;
	mode_register_t qcModeRegister = QC_MODE_REGISTER_0;
	
	// bitmask, meerdere configuratiebits gecombineerd worden tot ��n configuratiewaarde.
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
// Print met meegegeven string het channelnummer en tellerwaarde.
void QCEncodersShowCount(const char *idString)
{
	int32_t qcCountRegister = 0;
	uint8_t qcChannel = 0;

	for (qcChannel = 0; qcChannel <= QC_MAX_CHANNEL; qcChannel++)
	{
		qcCountRegister = qc_ReadCountRegister(qcChannel);
		vPrintString("%s channel %d: CNT = %8ld\n", idString, qcChannel, (long)qcCountRegister);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void QCEncodersClearCount(void)
// Zet alle Quadaturecounters op nul.
void QCEncodersClearCount(void)
{
	uint8_t qcChannel = 0;

	for (qcChannel = 0; qcChannel <= QC_MAX_CHANNEL; qcChannel++)
	{
		qc_ClearCountRegister(qcChannel);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Zet specifieke encoder op nul.
//qc_ClearCountRegister(uint8_t channel)

///////////////////////////////////////////////////////////////////////////////
// static void ReadMotorPositions(float motorPos_Rad[N_MOTORS])
//
// Lees motorenposities uit, en slaat deze op in de array "motorPos_Rad" in motor-radialen.
static float countsToRad = 0.00306796157577128245943617517898f; // = (2.0f * PI) / EncoderCountsPerRevolution;
static uint8_t mI = 0;
void ReadMotorPositions(float motorPos_Rad[N_MOTORS])
{
	for (mI = 0; mI < N_MOTORS; mI++)
	{
		motorPos_Rad[mI] = qc_ReadCountRegister(MotorQcChannel[mI]) * countsToRad;
	}
}
