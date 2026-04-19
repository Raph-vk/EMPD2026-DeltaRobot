/*
 * MotorControl.c
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#include "MotorControl.h"

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
Bool fault = true;
Bool isFaultInput(void)
{
	fault = true; // Assume safe
	fault = port_IsBitSet(BIT_NOOD);
	return fault;
}

///////////////////////////////////////////////////////////////////////////////
// void motor_DisplayStatus(void)
void motor_DisplayStatus(void)
{
	uint8_t portInValue = 0;
	uint8_t bitVal		= 0;
	Bool isSet			= false;
	
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

static Bool motor_IsHomeLimitActive(uint8_t motorIndex)
{
	if (motorIndex >= N_MOTORS)
	{
		return true;    // safe default
	}

	return port_IsBitSet(MotorHomeLimitBit[motorIndex]);
}

///////////////////////////////////////////////////////////////////////////////
// Check of er nog een homeswitch actief is.
static Bool anyHomeSwitchActive(void)
{
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		if (motor_IsHomeLimitActive(motorIndex))
		{
			return true;
		}
	}

	return false;
}


///////////////////////////////////////////////////////////////////////////////
// bool homeAllMotors(void)
//
// - Motoren die nog niet op home zitten, omhoog aansturen
// - Zodra limiet van die motor binnenkomt:
//      * encoder van die motor op nul
//      * motor in hold zetten
// - return true als alle 3 klaar zijn
static const float Home_uDac[N_MOTORS] = {4.0f, 4.0f, 4.0f};	// Homing-spanning per motor (omhoog gaande kopper).
static const float backoff_uDac[N_MOTORS] ={0.50f, 0.50f, 0.50f}; // Iets minder dan zwaartekracht
static const float slowHome_uDac[N_MOTORS] = {1.0f, 1.0f, 1.0f};	// Homing-spanning per motor (omhoog gaande kopper).
static const float Hold_uDac[N_MOTORS] ={0.60f, 0.60f, 0.60f};	// Hold-spanning per motor: ongeveer zwaartekracht compenseren.

static Bool HomingStarted = false;
static Bool readyPos[N_MOTORS] = { false, false, false };
static Bool allReady = false;
static Bool motorHomed[N_MOTORS] = { false, false, false };
static Bool allHomed = false;

static uint32_t i = 0;
 
Bool homeAllMotors(void)
{
	 motorIndex = 0;
	//motor_DisplayStatus();

	// Eerste keer: initialiseren / starten
	if (HomingStarted == false)
	{
		// DAC-spanning eerst op 0 forceren en alles false zetten
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
			readyPos[motorIndex] = false;
			motorHomed[motorIndex] = false;
		}
		allReady = false;
		allHomed = false;
		
		//Setup QCcounters.
		HomingStarted = true;
		QCEncodersSetup();
		
		// Voor iedere motor spanning opzetten, als deze nog niet op eindpositie is.
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			// Motor hoort niet al op limit te staan bij start:
			if (motor_IsHomeLimitActive(motorIndex))
			{
				//Zet op nul en houdt op positie.
				dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
				vPrintString("> Bij starten van Homing is er al een home-switch actief!\n");
				HomingStarted = false;
				ToState(STATE_FAULT);
				return false;
			}
			else
			{
				dac_SetOutputVoltage(MotorDacChannel[motorIndex], Home_uDac[motorIndex]);
			}
		}
	}

	///////////////////////////////////////////////
	// extra Check of systeem in fout staat
	if (isFaultInput())
	{
		vPrintString("> Er is een fout input actief!!\n");

		//Iedere motor spanning afnemen.
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
		}

		HomingStarted = false;
		return false;
	}


	///////////////////////////////////////////////////////////////////////////////
	// Eerst sensor opzoeken, als deze is op readyPositie zakken.
	if (!allReady)
	{
		allReady = true; // Veilig aannemen
	
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			// Als motor nog niet op ready Positie is.
			if (readyPos[motorIndex] == false)
			{
				// Als home switch actief is.
				if (motor_IsHomeLimitActive(motorIndex))
				{
					vPrintString("Motor: %u is at limit.\n", (unsigned int)motorIndex);
				
					// Motor een hangend-koppel geven
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], backoff_uDac[motorIndex]);
					readyPos[motorIndex] = true;
				}
				else
				{
					// Blijf homing-spanning uitsturen
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], Home_uDac[motorIndex]);
					allReady = false;
				}
			}
		}// Eind forloop

		//Als allReady nog True is en iedere arm ready
		if(allReady)
		{
			//Wachten tot all switches geen arm meer zien.
			if(anyHomeSwitchActive())
			{
				allReady = false;
				i = 0;
			}
			else
			{
				//Delay voor extra ruimte.
				if (i > 1000) //1000 = 1s delay (bij 1kHz triggering)
				{
					allReady = true;
					i = 0;
				}
				else{i++;}
			}	
		}
	}// einde AllReady statement
	///////////////////////////////////////////////////////////////////////////////
	// Langzaam nulpositie benaderen, en vasthouden als is bereikt.
	else if (!allHomed)
	{
		allHomed = true; // Veilig aannemen
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			// Als motor nog niet genult is.
			if (motorHomed[motorIndex] == false)
			{
				// Als home switch actief is.
				if (motor_IsHomeLimitActive(motorIndex))
				{
					vPrintString("Motor: %u is at Home.\n", (unsigned int)motorIndex);
					
					// Motor stil zetten en nullen
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
					qc_ClearCountRegister(MotorQcChannel[motorIndex]);

					// Motor een vasthoud-koppel geven
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], Hold_uDac[motorIndex]);
					motorHomed[motorIndex] = true;
				}
				else
				{
					// Blijf homing-spanning uitsturen
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], slowHome_uDac[motorIndex]);
					allHomed = false;
				}
			}
		}// Eind forloop
	}// einde AllHomed statement
	///////////////////////////////////////////////////////////////////////////////
	// Zekerheids check of echt alles klaar is
	else
	{	
		//Voor iedere motor
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			if (motorHomed[motorIndex] == false)
			{
				allHomed = false;
				break;
			}
		}
	}
	
	///////////////////////////////////////////////////////////////////////////////
	// Als alles genult is.
	if (allHomed)
	{
		HomingStarted = false;
		led_DisplayValue(0x0F);  //Alle ledjes op PCB aan.
	}

	return allHomed;
}//end-fuction
