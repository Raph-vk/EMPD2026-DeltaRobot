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
#include "QuadratureCounters.h"
#include "MachinePins.h"
//#include "ControlTask.h"



///////////////////////////////////////////////////////////////////////////////
// motor configuratie

static const uint8_t G_MotorDacChannel[N_MOTORS] ={0, 1, 2};
static const uint8_t G_MotorQcChannel[N_MOTORS] ={0, 1, 2};

// VUL HIER JOUW ECHTE LIMIT-BITS IN:
static const uint8_t G_MotorHomeLimitBit[N_MOTORS] = {M1_LIMIT, M2_LIMIT, M3_LIMIT};

// Homing-spanning per motor.
static const float Home_uDac[N_MOTORS] = {4.0f, 4.0f, 4.0f};

// Hold-spanning per motor: ongeveer zwaartekracht compenseren.
static const float Hold_uDac[N_MOTORS] ={0.60f, 0.60f, 0.60f};

static bool MotorHomed[N_MOTORS] = { false, false, false };
static bool HomingStarted = false;

/*
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
*/

///////////////////////////////////////////////////////////////////////////////
// bool motor_HasOverload(void)
bool motor_HasOverload(void)
{
	bool overload = true; // Assume safe

	overload = port_IsBitSet(BIT_NOOD);
	
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

	isSet = port_IsBitSet(BIT_NOOD);
	bitVal = isSet? 1 : 0;
	vPrintString("NOODSTOP / ESCON Overload: %d\n", bitVal);
	
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



///////////////////////////////////////////////////////////////////////////////
// bool homeAllMotors(void)
//
// - Motoren die nog niet op home zitten, omhoog aansturen
// - Zodra limiet van die motor binnenkomt:
//      * encoder van die motor op nul
//      * motor in hold zetten
// - return true als alle 3 klaar zijn

bool homeAllMotors(void)
{
	uint8_t motorIndex = 0;
	//motor_DisplayStatus();

	// Eerste keer: initialiseren / starten
	if (HomingStarted == false)
	{
		// DAC-spanning eerst op 0 forceren
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], 0.0f);
		}
		
		// Aandrijving inschakelen				
		HomingStarted = true;
		QCEncodersSetup();
		//motor_EnableESCONController();
		
		// Voor iedere motor spanning opzetten, als deze nog niet op eindpositie is.
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			MotorHomed[motorIndex] = false;

			// Als motor al op limit staat bij start:
			if (motor_IsHomeLimitActive(motorIndex))
			{
				//Zet op nul en houdt op positie.
				qc_ClearCountRegister(G_MotorQcChannel[motorIndex]);
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], Hold_uDac[motorIndex]);
				MotorHomed[motorIndex] = true;
			}
			else
			{
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], Home_uDac[motorIndex]);
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
			MotorHomed[motorIndex] = false;
		}

		HomingStarted = false;
		// motor_DisableESCONController();  // (gebeurd al in emergencyinterrupt)
		return false;
	}


	
	// Homing per motor afhandelen
	bool allHomed = true; //assume safe
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		if (MotorHomed[motorIndex] == false)
		{
			if (motor_IsHomeLimitActive(motorIndex))
			{
				vPrintString("Motor: %u is at limit.\n", (unsigned int)motorIndex);
				// Eerst even stil / veilig schakelen
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], 0.0f);

				// Encoder van die motor op nul
				qc_ClearCountRegister(G_MotorQcChannel[motorIndex]);

				// Daarna hold-koppel geven
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], Hold_uDac[motorIndex]);

				MotorHomed[motorIndex] = true;
			}
			else
			{
				// Blijf homing-spanning uitsturen
				dac_SetOutputVoltage(G_MotorDacChannel[motorIndex], Home_uDac[motorIndex]);
				allHomed = false;
			}
		}
	}

	// Check of echt alles klaar is
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		if (MotorHomed[motorIndex] == false)
		{
			allHomed = false;
			break;
		}
	}

	// Als alles gehomed is
	if (allHomed)
	{
		HomingStarted = false;
		led_DisplayValue(0x0F);  //Alle ledjes aan.
	}
		
	return allHomed;
}
