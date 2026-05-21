/*
 * MotorControl.c
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#include "MotorControl.h"
#include "Regelaar.h"

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
// bool motor_IsHomeLimitActive(uint8_t motorIndex)

Bool motor_IsHomeLimitActive(uint8_t index)
{
	if (index >= N_MOTORS)
	{
		vPrintString(">INDEX MOTOR OUT OF RANGE!!");
		return true;    // safe default
	}
	return !port_IsBitSet(MotorHomeLimitBit[index]);
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
// VOLTAGE negative is moving UP, positive is moving down
static const float readyArmAngle = -0.5235988f; //30graden hoek in rad, gewenste armpositie voordat homing benadering begint
static const float Home_uDac = -1.0f;	// Homing-spanning per motor (omhoog gaande kopper).
static const float slowHome_uDac = 0.5f;	// Homing-spanning per motor (omhoog gaande kopper).
static const float Hold_uDac = 2.0f;

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
				dac_SetOutputVoltage(MotorDacChannel[motorIndex], Home_uDac);
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
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], Home_uDac);
					allReady = false;
				}
			}
			// Motor op ReadyPositie
			else
			{
				// Op positie behouden met PID
				float error = motorPos_Rad[motorIndex] - readyArmAngle;
				motorControlOutput = Hold_uDac + PIDregelaar(motorIndex, error); //motorspanning bepalen met PID-regelaar
				motorControlOutput = constrain(motorControlOutput, DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE); // ongeveer +/-10V begrenzen.
				dac_SetOutputVoltage(MotorDacChannel[motorIndex], motorControlOutput);
			}
		}// Eind per Motor For-loop

		//Als alles Ready is, naar Backoff Positie gaan tot die positie bereikt is.
		if(allReady)
		{
			float HoldPos[N_MOTORS] = {readyArmAngle, readyArmAngle, readyArmAngle};
			allReady = HoldPosition(HoldPos);
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
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], slowHome_uDac);
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
