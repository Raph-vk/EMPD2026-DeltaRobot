/*
 * MotorControl.c
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "LEDLib.h"
#include "PortIOLib.h"
#include "DAC4921Lib.h"
#include "QC7366Lib.h" // LS7366R encoder counter


///////////////////////////////////////////////////////////////////////////////
// application includes
#include "MotorControl.h"
//#include "ControlTask.h"



///////////////////////////////////////////////////////////////////////////////
// motor configuratie

static const uint8_t G_MotorDacChannel[N_MOTORS] ={0, 1, 2};
static const uint8_t G_MotorQcChannel[N_MOTORS] ={0, 1, 2};

// VUL HIER JOUW ECHTE LIMIT-BITS IN:
static const uint8_t G_MotorHomeLimitBit[N_MOTORS] = {M1_LIMIT, M2_LIMIT, M3_LIMIT};

// Homing-spanning per motor.
// Zet het teken per motor goed afhankelijk van draairichting!
static const float G_MotorHomeVoltage[N_MOTORS] = {4.0f, 4.0f, 4.0f};

// Hold-spanning per motor: ongeveer zwaartekracht compenseren.
// Deze waardes moet je praktisch afregelen.
static const float G_MotorHoldVoltage[N_MOTORS] ={0.60f, 0.60f, 0.60f};

static bool G_MotorHomed[N_MOTORS] = { false, false, false };
static bool G_HomingStarted = false;

// dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], voltage);


///////////////////////////////////////////////////////////////////////////////
// void QCEncodersSetup(void)
// Zet de juiste instellingen voor de quadratureCounters.

void QCEncodersSetup(void)
{
	uint8_t qcChannel     = 0;
	uint8_t	qcDefaultMode = 0;
	mode_register_t qcModeRegister = QC_MODE_REGISTER_0;
	
	// bitmask, meerdere configuratiebits gecombineerd worden tot één configuratiewaarde.
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
// void QCEncodersClearCount(uint8_t qcChannel)
// Zet 1 specifieke quadraturecounter op nul.
void QCEncodersClearCount(uint8_t qcChannel)
{
	if (qcChannel <= QC_MAX_CHANNEL)
	{
		qc_ClearCountRegister(qcChannel);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void motor_EnableESCONController(void)
//
//enable ESCON controller via output port bit 0

void motor_EnableESCONController(void)
{
	port_SetBit(ESCON_ENABLE, true);
}

///////////////////////////////////////////////////////////////////////////////
// void motor_DisableESCONController(void)
//
// disable ESCON controller via output port bit 0

void motor_DisableESCONController(void)
{
	port_SetBit(ESCON_ENABLE, false);
}

///////////////////////////////////////////////////////////////////////////////
// bool motor_HasOverload(void)
bool motor_HasOverload(void)
{
	bool overload = true; // Assume safe

	overload = port_IsBitSet(ESCON_OVERLOAD);
	
	return overload;
}

///////////////////////////////////////////////////////////////////////////////
// void motor_DisplayStatus(void)
void motor_DisplayStatus(void)
{
	uint8_t portInValue = 0;
	uint8_t bitVal		= 0;
	bool isSet			= false;
	
	// non-inverting input port, pull-up resistors
	
	portInValue = port_GetInput();
	
	led_DisplayValue(portInValue >> 1);	// using bits 1..4

	vPrintString("digital input = 0x%02x\n", portInValue);
	
	isSet = port_IsBitSet(M1_LIMIT);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit M1:    %d\n", bitVal);

	isSet = port_IsBitSet(M2_LIMIT);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit M2:    %d\n", bitVal);

	isSet = port_IsBitSet(M3_LIMIT);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit M3:     %d\n", bitVal);

	isSet = port_IsBitSet(ESCON_OVERLOAD);
	bitVal = isSet? 1 : 0;
	vPrintString("ESCON Overload: %d\n", bitVal);
	
	vPrintString("\n");
}
		
///////////////////////////////////////////////////////////////////////////////
// static bool motor_IsHomeLimitActive(uint8_t motorIndex)

static bool motor_IsHomeLimitActive(uint8_t motorIndex)
{
	if (motorIndex >= N_MOTORS)
	{
		return true;    // safe default
	}

	return port_IsBitSet(G_MotorHomeLimitBit[motorIndex]);
}

/*
///////////////////////////////////////////////////////////////////////////////
// void MotorHold(uint8_t motorIndex)
//
// Behoud motor op ongeveer zwaartekrachtkoppel. 
void MotorHold(uint8_t motorIndex)
{
	if (motorIndex < N_MOTORS)
	{
		dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], G_MotorHoldVoltage[motorIndex]);
	}
}
*/


///////////////////////////////////////////////////////////////////////////////
// bool motors_Move(void)
//
// - Motoren die nog niet op home zitten, omhoog aansturen
// - Zodra limiet van die motor binnenkomt:
//      * encoder van die motor op nul
//      * motor in hold zetten
// - return true als alle 3 klaar zijn

bool homeAllMotors(void)
{
	uint8_t motorIndex = 0;
	bool allHomed = true;
	
	//motor_DisplayStatus();

	// Eerste keer: initialiseren / starten
	if (G_HomingStarted == false)
	{
		// DAC-spanning eerst op 0 forceren
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], 0.0f);
		}
		
		// Aandrijving inschakelen				
		G_HomingStarted = true;
		QCEncodersSetup();
		motor_EnableESCONController();
		
		// Voor iedere motor spanning opzetten, als deze nog niet op eindpositie is.
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			G_MotorHomed[motorIndex] = false;

			// Als motor al op limit staat bij start:
			if (motor_IsHomeLimitActive(motorIndex))
			{
				//Zet op nul en houdt op positie.
				QCEncodersClearCount(G_MotorQcChannel[motorIndex]);
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], G_MotorHoldVoltage[motorIndex]);
				G_MotorHomed[motorIndex] = true;
			}
			else
			{
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], G_MotorHomeVoltage[motorIndex]);
			}
		}
	}


	// Een motor een overload signaal geeft
	if (motor_HasOverload())
	{
		vPrintString("> Motor has Overloaded!!\n");

		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], 0.0f);
			G_MotorHomed[motorIndex] = false;
		}

		G_HomingStarted = false;
		// motor_DisableESCONController();  // (gebeurd al in emergencyinterrupt)
		return false;
	}

	// Homing per motor afhandelen
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		if (G_MotorHomed[motorIndex] == false)
		{
			if (motor_IsHomeLimitActive(motorIndex))
			{
				vPrintString("Motor: %u is at limit.\n", (unsigned int)motorIndex);
				// Eerst even stil / veilig schakelen
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], 0.0f);

				// Encoder van die motor op nul
				QCEncodersClearCount(G_MotorQcChannel[motorIndex]);

				// Daarna hold-koppel geven
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], G_MotorHoldVoltage[motorIndex]);

				G_MotorHomed[motorIndex] = true;
			}
			else
			{
				// Blijf homing-spanning uitsturen
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], G_MotorHomeVoltage[motorIndex]);
				allHomed = false;
			}
		}
	}

	// Check of echt alles klaar is
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		if (G_MotorHomed[motorIndex] == false)
		{
			allHomed = false;
			break;
		}
	}

	if (allHomed)
	{
		G_HomingStarted = false;
		led_DisplayValue(0x0F);  //Alle ledjes aan.
	}

	return allHomed;
}