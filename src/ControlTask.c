/*
 * ControlTask.c
 *
 * Created: 10/04/2026
 * Author: Raph van Koeveringe
 */
///////////////////////////////////////////////////////////////////////////////
#include "ControlTask.h"
///////////////////////////////////////////////////////////////////////////////

// file globals
static SystemState_t state = STATE_INIT;

///////////////////////////////////////////////////////////////////////////////
// void ToState(newState)
//
// Makes correctly the new state and communicates that.
void ToState(SystemState_t newState)
{
	if (state != newState)
	{
		state = newState;
		xQueueOverwrite(handle_StateQueue, &state);
	}
}


///////////////////////////////////////////////////////////////////////////////
// bool IsFaultInputActive(void)
//
// Returns true if one of the fault inputs is still active.
// PIN_NOOD is fail-safe active-low: high level = OK, low level = emergency.
Bool IsFaultInputActive(void)
{
	return !port_IsBitSet(BIT_NOOD);
}

///////////////////////////////////////////////////////////////////////////////
// void ClockInterruptHandler(uint32_t id, uint32_t mask)
//
// invoked on every clock tick (1 ms) of the external hardware clock

void ClockInterruptHandler(uint32_t id, uint32_t mask)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (handle_ControlTask != NULL)
	{
		vTaskNotifyGiveFromISR(handle_ControlTask, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

///////////////////////////////////////////////////////////////////////////////
// void EmergencyInterruptHandler(uint32_t id, uint32_t mask)
//
// emergency input interrupt
void EmergencyInterruptHandler(uint32_t id, uint32_t mask)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// Direct veilig maken
	//motor_DisableESCONController();

	if (handle_EmergencySemaphore != NULL)
	{
		xSemaphoreGiveFromISR(handle_EmergencySemaphore, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

///////////////////////////////////////////////////////////////////////////////
/* void ControlTask(void *pvParameters)

State machine volgens ontwerp:
WAIT -> HOMING -> READY -> RUNNING -> READY
Bij fout/noodinput altijd naar FAULT.
Vanuit FAULT met reset + geen foutsignaal terug naar WAIT.

*/
void ControlTask(void *pvParameters)
{
	EventBits_t buttonBits = 0;
	const BaseType_t clearAllbits  = pdTRUE;		// FALSE = bits blijven staan na continue, TRUE = bits worden gewist.
	const BaseType_t waitForAnyBit= pdFALSE;			// FALSE = wacht totdat één v/d bits is gezet, TRUE wacht op ALLE bits.

	EventBits_t threadBits = 0;
	const BaseType_t stayAllbits  = pdFALSE;		// FALSE = bits blijven staan na continue, TRUE = bits worden gewist.
	const BaseType_t waitForAllbits = pdTRUE;			// FALSE = wacht totdat één v/d bits is gezet, TRUE wacht op ALLE bits.
	
	const TickType_t ticksToWait	  = portMAX_DELAY;	// Maximale wachttijd, portMAX_DELAY = onbeperkt wachten.
	const float RustPostitie[N_MOTORS] = {20.0f,20.0f,20.0f};
		
	Bool homingAllMotorsDone = false;
	Bool sequenceDone  = false;

	uint32_t flags = 0;
	
	vPrintString("> starting ControlTask.\n");

	ToState(STATE_INIT);
	
	// schakeld direct de motoren uit.
	//motor_DisableESCONController();
	
	// Bij opkomend signaal in PIN, run ClockInterruptHandler.
	flags = PIO_IT_RISE_EDGE; // PIO_IT_FALL_EDGE
	interrupt_AttachHandler(ClockInterruptHandler, PIN_CLOCK, flags);
	
	// Bij wegvallen van het noodsignaal, run EmergencyInterruptHandler.
	flags = PIO_IT_LOW_LEVEL;
	interrupt_AttachHandler(EmergencyInterruptHandler, PIN_NOOD, flags);

	// wait for all tasks to get up and running:
	vPrintString("> ControlTask waiting for helper tasks...\n");
	threadBits = xEventGroupWaitBits(handle_ThreadEventGroup, BIT_0 | BIT_1, stayAllbits, waitForAllbits, ticksToWait);
	
	// Helper taken zijn ready
	vPrintString("> helper tasks running, ControlTask started, event group = 0x%04lx\n", (unsigned long)threadBits);
	ToState(STATE_WAIT);
	
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


		// Emergency check via Semaphore
		if (xSemaphoreTake(handle_EmergencySemaphore, 0) == pdTRUE)
		{
			vPrintString("> EMERGENCY INTERRUPT RECIEVED\n");
			ToState(STATE_FAULT);
		}

		switch (state)
		{
			/////////////////////////////////////////////////////////////////////
			case  STATE_WAIT:
			{
				// Wachten op start-of resetknop
				if ((buttonBits & EVT_START_BUTTON) || (buttonBits & EVT_RESET_BUTTON))
				{

					// Naar HomingState schakelen.
					vPrintString("> WAIT -> HOMING ( Start-of Resetknop is ontvangen).\n");
					homingAllMotorsDone = false;
					ToState(STATE_HOMING);
				}
				taskSleep(10);
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_HOMING:
			{
				taskSleep(1);
				homingAllMotorsDone = homeAllMotors(); //<- MotorControl.c
				
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
				
				HoldPosition(RustPostitie);

				// Startknop -> runnen
				if (buttonBits & EVT_START_BUTTON)
				{
					InitSequence();
					vPrintString("> READY -> RUNNING (Startknop ontvangen.)\n");
					ToState(STATE_RUNNING);
				}

				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_RUNNING:
			{
				// Sequence draaien op control tick
				ulTaskNotifyTake(pdTRUE, ticksToWait);
				
				// Uitvoeren van bepaalde stappen.
				sequenceDone = RunSequence();

				// Stopknop -> terug naar READY
				if (buttonBits & EVT_STOP_BUTTON)
				{
					HoldCurrentPosition(0);
					vPrintString("> RUNNING -> READY (stopknop ontvangen).\n");
					ToState(STATE_READY);
				}
				// Cyclus klaar -> terug naar READY
				else if (sequenceDone)
				{
					HoldCurrentPosition(0);
					vPrintString("> RUNNING -> READY (cyclus voldaan)\n");
					ToState(STATE_READY);
				}
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_FAULT:
			{
				//Iedere motor geen kracht forceren.
				for (uint8_t motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
				{
					dac_SetOutputVoltage(MotorDacChannel[motorIndex], 0.0f);
				}

				taskSleep(100);

				// Alleen uit fault als reset is gedrukt EN foutsignaal weg is
				if (buttonBits & EVT_RESET_BUTTON)
				{
					if (!IsFaultInputActive())
					{
						vPrintString("> FAULT cleared -> WAIT (reset ontvangen, foutsignaal weg)\n");
						ToState(STATE_WAIT);
					}
					else
					{
						vPrintString("> RESET ontvangen, maar fault input is nog actief\n");
					}
				}
				break;
			}

			/////////////////////////////////////////////////////////////////////
			default: // Onbekende Status?
			{
				ToState(STATE_FAULT);
				break;
			}
			
		}//End-SwitchCase
	
	}//End-WhileLoop
	
	/* Should never get here */
	vTaskDelete(NULL);
}//End-ControlTask
