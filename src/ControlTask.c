/*
 * ControlTask.c
 *
 * Created: 10/04/2026
 * Author: Raph van Koeveringe
 */
#include "ControlTask.h"
///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board
#include "DeviceIOLib.h"
#include "InterruptLib.h"
#include "bits.h"
#include "DAC4921Lib.h" //voor setDACoutput
#include "PortIOLib.h"
#include "QC7366Lib.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "Homing.h"
#include "MotorControl.h" // <- regelaarINIT
#include "InputHandlerTask.h"
#include "MotionEngine.h"
#include "ApplicationTasks.h"
#include "MachinePins.h"
#include "temp.h"

///////////////////////////////////////////////////////////////////////////////
// file globals
static SystemState_t state = STATE_INIT;

///////////////////////////////////////////////////////////////////////////////
// static const char *StateToString(SystemState_t systemState)
/*
 * Zet een SystemState_t om naar een leesbare tekst voor debug-uitvoer.
 * Invoer: systemState is de state die vertaald moet worden.
 * Uitvoer: pointer naar een constante string met de statenaam.
 */
static const char *StateToString(SystemState_t systemState)
{
	switch (systemState)
	{
		case STATE_INIT:    return "INIT";
		case STATE_WAIT:    return "WAIT";
		case STATE_HOMING:  return "HOMING";
		case STATE_READY:   return "READY";
		case STATE_PAUSE:   return "PAUSE";
		case STATE_RUNNING: return "RUNNING";
		case STATE_FAULT:   return "FAULT";
		default:            return "UNKNOWN";
	}
}

///////////////////////////////////////////////////////////////////////////////
// void ToState(SystemState_t newState)
/*
 * Schakelt de state machine naar een nieuwe state wanneer deze veranderd is.
 * Invoer: newState is de gewenste nieuwe systeemstatus.
 * Uitvoer: geen returnwaarde; de state queue en debug-uitvoer worden bijgewerkt.
*/
void ToState(SystemState_t newState)
{
	if (state != newState)
	{
		state = newState;
		xQueueOverwrite(handle_StateQueue, &state);
		vPrintString("> Switch to state = %s\n", StateToString(state));
	}
}


///////////////////////////////////////////////////////////////////////////////
// bool InNoodsituatie(void)
/*
 * Leest de directe status van de noodinput.
 * PIN_NOOD is fail-safe actief-laag: hoog = OK, laag = nood.
 * Uitvoer: true wanneer BIT_NOOD hoog is, false wanneer de input laag is.
*/
bool InNoodsituatie(void)
{
	return !port_IsBitSet(BIT_NOOD);
}

///////////////////////////////////////////////////////////////////////////////
// void ClockInterruptHandler(uint32_t id, uint32_t mask)
/*
 * Wordt bij iedere externe 1 ms clockpuls aangeroepen.
 * Invoer: id en mask komen uit de interruptlaag en worden hier niet gebruikt.
 * Uitvoer: geen returnwaarde; geeft ControlTask een task notification.
*/ 
static void ClockInterruptHandler(uint32_t id, uint32_t mask)
{
	// Als ControlTask bestaat, geef een task notification.
	if (handle_ControlTask != NULL)
	{
		vTaskNotifyGiveFromISR(handle_ControlTask, NULL);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void NoodInterruptHandler(uint32_t id, uint32_t mask)
/*
 * Wordt aangeroepen wanneer de noodinput een vallende flank geeft.
 * Invoer: id en mask komen uit de interruptlaag en worden hier niet gebruikt.
 * Uitvoer: geen returnwaarde; geeft de nood-semaphore vrij vanuit de ISR.
 */
static void NoodInterruptHandler(uint32_t id, uint32_t mask)
{
	// NoodSemaphore vrijgeven bij trigger
	if (handle_NoodSemaphore != NULL)
	{
		//xSemaphoreGiveFromISR(handle_NoodSemaphore, 0);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void ControlTask(void *pvParameters)
/*
 * Hoofdtaak voor de systeem-state-machine van de robot.
 * Verwerkt knoppen, noodsituaties, homing, stilstand en bewegingen.
 * Invoer: pvParameters wordt niet gebruikt.
 * Uitvoer: geen returnwaarde; de taak blijft draaien tot het systeem stopt.
*/
void ControlTask(void *pvParameters)
{
	///////////////////////////////////////////////////////////////////////////////
	//Taak declaraties
	EventBits_t buttonBits = 0;
	const BaseType_t clearAllbits  = pdTRUE;		// FALSE = bits blijven staan na continue, TRUE = bits worden gewist.
	const BaseType_t waitForAnyBit= pdFALSE;			// FALSE = wacht totdat één v/d bits is gezet, TRUE wacht op ALLE bits.

	//EventBits_t threadBits = 0;
	const BaseType_t stayAllbits  = pdFALSE;		// FALSE = bits blijven staan na continue, TRUE = bits worden gewist.
	const BaseType_t waitForAllbits = pdTRUE;			// FALSE = wacht totdat één v/d bits is gezet, TRUE wacht op ALLE bits.
	
	const TickType_t ticksToWait	  = portMAX_DELAY;	// Maximale wachttijd, portMAX_DELAY = onbeperkt wachten.
	// const float RustPostitie[N_MOTORS] = {20.0f,20.0f,20.0f};
	
	bool homingAllMotorsDone = false;
	bool sequenceDone  = false;

	float gemetenStroom = 0.0f;
	BaseType_t hasCurrentSample = pdFALSE;
	
	float rustPositie[N_MOTORS] = {0.0f,0.0f,0.0f};
	///////////////////////////////////////////////////////////////////////////////
	vPrintString("> starting ControlTask.\n");

	ToState(STATE_INIT);
	///////////////////////////////////////////////////////////////////////////////
	//opstart instellingen
	
	//ALLES UITSCHAKELEN.
	port_SetBit(BIT_GRIPPER, false);
	port_SetBit(BIT_VOEDING, false);
	for (uint8_t qc_channel = 0; qc_channel < QC_MAX_CHANNEL; qc_channel++)
	{
		dac_SetOutputVoltage(qc_channel, 0.0f);
	}

	RunningLoopTimer_Init();
	Regelaar_INIT();
	
	// Bij opkomend signaal in PIN, run ClockInterruptHandler.
	interrupt_AttachHandler(ClockInterruptHandler, PIN_CLOCK, PIO_IT_RISE_EDGE);
	interrupt_Disable(PIN_CLOCK);
	interrupt_Enable(PIN_CLOCK);

	// PIN_NOOD is fail-safe, pinSet = systemOK, not set = Nood
	interrupt_AttachHandler(NoodInterruptHandler, PIN_NOOD, PIO_IT_FALL_EDGE);
	interrupt_Disable(PIN_NOOD);
	interrupt_Enable(PIN_NOOD);


	// wait for all tasks to get up and running:
	vPrintString("> ControlTask wachtende op helper taken...\n");
	xEventGroupWaitBits(handle_ThreadEventGroup, BIT_0 | BIT_1, stayAllbits, waitForAllbits, ticksToWait);
	
	// Helper taken zijn ready
	vPrintString("> helper taken draaien, ControlTask wordt nu ook gestart.\n");
	
	// Controleer al in nood is.
	if( InNoodsituatie() )
	{
		vPrintString("> Systeem is al in noodSitautie!\n");
		ToState(STATE_FAULT);
	}
	else
	{
		vPrintString("> Systeem prima.\n");
		ToState(STATE_WAIT);
	}
	
	///////////////////////////////////////////////////////////////////////////
	// oneindige loop
	while (true)
	{
		// Lees uit of er een knop ingedrukt is.
		buttonBits = xEventGroupWaitBits(handle_ButtonEventGroup,
			EVT_START_BUTTON | EVT_STOP_BUTTON | EVT_RESET_BUTTON,
			clearAllbits,	// ontvangen bits wissen
			waitForAnyBit,	// één van de bits is genoeg
			0				// niet blokkeren
		);

		// Nood check via Semaphore, of als fout actief is, of als stroom te hoog oploopt.
		hasCurrentSample = xQueuePeek(handle_stroomQueue, &gemetenStroom, 0);
		
		if ( xSemaphoreTake(handle_NoodSemaphore, 0) == pdTRUE || (hasCurrentSample == true && gemetenStroom >= (maxStroom-1.0) ) ) //
		{
			vPrintString("> handle_NoodSemaphore VRIJGEGEVEN!\n");
			if (state != STATE_FAULT)
			{
				ToState(STATE_FAULT);

				if(InNoodsituatie())
				{
					vPrintString("> Noodinput actief!\n");
				}
				else if (gemetenStroom >= maxStroom)
				{
					port_SetBit(BIT_VOEDING, true);
					vPrintString("> NOOD: Gemeten stroom te hoog, %.2f!\n", gemetenStroom);
				}
			}
		}

		switch (state)
		{
			/////////////////////////////////////////////////////////////////////
			case  STATE_WAIT:
			{
				// Wachten op start-of resetknop
				if ((buttonBits & EVT_START_BUTTON)) //|| (buttonBits & EVT_RESET_BUTTON)
				{
					// Naar HomingState schakelen.
					vPrintString("> WAIT -> HOMING ( Start-of Resetknop is ontvangen).\n");
					homingAllMotorsDone = false;
					ToState(STATE_HOMING);
				}

				//ALLES UITFORCEREN.
				port_SetBit(BIT_GRIPPER, false);
				for (uint8_t motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
				{
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
				}

				taskSleep(10);
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_HOMING:
			{
				// 1kHz take
				ulTaskNotifyTake(pdTRUE, ticksToWait);

				homingAllMotorsDone = homeAllMotors(); //<- Homing.c
				//homingAllMotorsDone = true;
				
				if (homingAllMotorsDone)
				{
					HoldCurrentPosition(0.0f);
					
					vPrintString("> HOMING complete -> READY\n");
					ToState(STATE_READY);
				}
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_READY:
			{
				// Op vaste positie regelen op iedere control tick
				ulTaskNotifyTake(pdTRUE, ticksToWait);
				
				HoldPosition(rustPositie);

				// Startknop -> runnen
				if (buttonBits & EVT_START_BUTTON)
				{
					RunningLoopTimer_ResetWindow();	//ONLY for 1kHz loop check		

					SequenceRESET();
					//tijdelijk naar homing, normaliter naar; RUNNING
					vPrintString("> READY -> RUNNING (Startknop ontvangen.)\n");
					ToState(STATE_RUNNING);
				}

				break;
			}
			/////////////////////////////////////////////////////////////////////
			case  STATE_PAUSE:
			{
				// Op vaste positie regelen op iedere control tick
				ulTaskNotifyTake(pdTRUE, ticksToWait);
				
				HoldPosition(rustPositie); //Change to current/last position

				// Startknop -> runnen
				if (buttonBits & EVT_START_BUTTON)
				{
					vPrintString("> READY -> RUNNING (Startknop ontvangen.)\n");
					ToState(STATE_RUNNING);
					RunningLoopTimer_ResetWindow();
				}
				// Resetknop -> READY
				if (buttonBits & EVT_RESET_BUTTON)
				{
					vPrintString("> RESET -> READY (Startknop ontvangen.)\n");
					ToState(STATE_READY);
				}

				break;
			}


			/////////////////////////////////////////////////////////////////////
			case  STATE_RUNNING:
			{
				RunningLoopTimer_End(); //ONLY for 1kHz loop check
				
				// Sequence draaien op control tick
				ulTaskNotifyTake(pdTRUE, ticksToWait);
				
				RunningLoopTimer_Begin();

				// Uitvoeren van bepaalde stappen.
				sequenceDone = RunSequence();

				// Stopknop -> terug naar READY
				if (buttonBits & EVT_STOP_BUTTON)
				{
					//HoldCurrentPosition(0);
					vPrintString("> RUNNING -> READY (stopknop ontvangen).\n");
					ToState(STATE_PAUSE);
				}
				// Cyclus klaar -> terug naar READY
				else if (sequenceDone)
				{
					//HoldCurrentPosition(0);
					vPrintString("> RUNNING -> READY (cyclus voldaan).\n");
					ToState(STATE_READY);
				}
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_FAULT:
			{
				//Iedere motor geen kracht forceren.
				port_SetBit(BIT_GRIPPER, false);
				for (uint8_t motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
				{
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
				}


				// Alleen uit fault als reset is gedrukt EN foutsignaal weg is
				if (buttonBits & EVT_RESET_BUTTON)
				{
					if (!InNoodsituatie())
					{
						vPrintString("> FAULT cleared -> WAIT (reset ontvangen, signaal ook actief)\n");
						ToState(STATE_WAIT);
					}
					else
					{
						vPrintString("> RESET ontvangen, maar PIN_NOOD heeft nog steeds geen signaal!\n");
					}
				}
				
				taskSleep(1);
				
				break;
			}

			/////////////////////////////////////////////////////////////////////
			default: // Onbekende Status?
			{
				vPrintString("> ONBEKENDE STATUS!!!\n");
				ToState(STATE_FAULT);
				break;
			}
			
		}//End-SwitchCase
	
	}//End-WhileLoop
	
	/* Should never get here */
	vTaskDelete(NULL);
}//End-ControlTask
