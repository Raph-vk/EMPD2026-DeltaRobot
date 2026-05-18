/*
 * MotorControl.c
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#include "MotorControl.h"

///////////////////////////////////////////////////////////////////////////////
// globals
//static const float degToRad = 0.01745329251994329576923690768489f; //PI / 180.0f; // conversiefactor van graden naar radialen
//static const float RadToDeg = 57.295779513082320876798154814105f; // 180.0f / PI; // conversiefactor van radialen naar graden

static float motorPos_Rad[N_MOTORS] = {0.0f, 0.0f, 0.0f};// Vast te houden motorpositie [rad]
static uint8_t motorIndex;
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
// void motor_DisplayStatus(void)
void motor_DisplayStatus(void)
{
	//uint8_t portInValue = 0;
	uint8_t bitVal		= 0;
	Bool isSet			= false;
	
	// non-inverting input port, pull-up resistors
	
	//portInValue = port_GetInput();
	
	//led_DisplayValue(portInValue >> 1);	// using bits 1..4

	//vPrintString("digital input = 0x%02x\n", portInValue);
	
	isSet = port_IsBitSet(BIT_M1_HOME);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit M1:    %d\n", bitVal);

	isSet = port_IsBitSet(BIT_M2_HOME);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit M2:    %d\n", bitVal);

	isSet = port_IsBitSet(BIT_M3_HOME);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit M3:     %d\n", bitVal);

	isSet = port_IsBitSet(BIT_NOOD);
	bitVal = isSet? 1 : 0;
	vPrintString("NOODSTOP / ESCON Overload: %d\n", bitVal);
	
	vPrintString("\n");
}
		
///////////////////////////////////////////////////////////////////////////////
// bool motor_IsHomeLimitActive(uint8_t motorIndex)

Bool motor_IsHomeLimitActive(uint8_t index)
{
	if (index >= N_MOTORS)
	{
		return true;    // safe default
	}
	return port_IsBitSet(MotorHomeLimitBit[index]);
}

///////////////////////////////////////////////////////////////////////////////
// Check of er nog een homeswitch actief is. 
Bool anyHomeSwitchActive(void)
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

static const float readyArmAngle[N_MOTORS] = {-10.0f, -10.0f, -10.0f}; //degrees, gewenste armpositie voordat homing benadering begint
static const float Home_uDac[N_MOTORS] = {4.0f, 4.0f, 4.0f};	// Homing-spanning per motor (omhoog gaande kopper).
static const float slowHome_uDac[N_MOTORS] = {1.0f, 1.0f, 1.0f};	// Homing-spanning per motor (omhoog gaande kopper).
static const float Hold_uDac = 0.5f;

static float motorControlOutput = 0.0f;
static Bool HomingStarted = false;
static Bool readyPos[N_MOTORS] = { false, false, false };
static Bool allReady = false;
static Bool motorHomed[N_MOTORS] = { false, false, false };
static Bool allHomed = false;
 
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
				//fault trigger?
				HomingStarted = false;
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
	if (InNoodsituatie())
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
		ReadMotorPositions(motorPos_Rad);
		for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
		{
			// Als motor nog niet op ready Positie is.
			if (readyPos[motorIndex] == false)
			{
				// Als home switch actief is.
				if (motor_IsHomeLimitActive(motorIndex))
				{
					// Motor stil zetten en nullen
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
					qc_ClearCountRegister(MotorQcChannel[motorIndex]);

					vPrintString("Motor: %u is at limit.\n", (unsigned)motorIndex);

					readyPos[motorIndex] = true;
				}
				else
				{
					// Blijf homing-spanning uitsturen
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], Home_uDac[motorIndex]);
					allReady = false;
				}
			}
			// Motor op ReadyPositie
			else
			{
				// Op positie behouden met PID
				motorControlOutput = Hold_uDac + PID_Controller(motorPos_Rad[motorIndex]);
				motorControlOutput = constrain(motorControlOutput, DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE); // ongeveer +/-10V begrenzen.
				dac_SetOutputVoltage(MotorDacChannel[motorIndex], motorControlOutput);
			}
		}// Eind per Motor For-loop

		//Als alles Ready is, naar Backoff Positie gaan tot die positie bereikt is.
		if(allReady)
		{
			allReady = HoldPosition(readyArmAngle);
		}
	}// einde AllReady statement
	///////////////////////////////////////////////////////////////////////////////
	// Langzaam nulpositie benaderen, en vasthouden als is bereikt.
	else if (!allHomed && allReady)
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
					
					// Motor in hold zetten
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], Hold_uDac);
					motorHomed[motorIndex] = true;
				}
				else // Blijf langzaam naar home positie bewegen
				{
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
