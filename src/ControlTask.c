/*
 * ControlTask.c
 *
 * Created: 11/04/2026
 * Author: Raph van Koeveringe
 */

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DeviceIOLib.h"
#include "InterruptLib.h"
#include "bits.h"

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "PositionControllerLoad.h"
#include "MotorControl.h"
#include "QuadratureCounters.h"
#include "ButtonHandlerTask.h"
#include "ControlTask.h"
#include "ApplicationTasks.h"

///////////////////////////////////////////////////////////////////////////////
// file globals

static SemaphoreHandle_t TimerInterruptSemaphore = NULL;

///////////////////////////////////////////////////////////////////////////////
// void ClockInterruptHandler(uint32_t id, uint32_t mask)
//
// invoked on every clock tick (1 ms) of the external hardware clock

void ClockInterruptHandler(uint32_t id, uint32_t mask)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (TimerInterruptSemaphore != NULL)
	{
		xSemaphoreGiveFromISR(TimerInterruptSemaphore, &xHigherPriorityTaskWoken);
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
	vPrintString("> EMERGENCY INTERRUPT!!\n");
	// Direct veilig maken
	motor_DisableESCONController();

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
	SystemState_t state =  STATE_INIT;
	
	EventBits_t buttonBits = 0;
	BaseType_t clearAllbits  = pdTRUE;		// FALSE = bits blijven staan na continue, TRUE = bits worden gewist.
	BaseType_t dontWaitForAllbits = pdFALSE;			// FALSE = wacht totdat één v/d bits is gezet, TRUE wacht op ALLE bits.

	EventBits_t threadBits = 0;
	BaseType_t stayAllbits  = pdFALSE;		// FALSE = bits blijven staan na continue, TRUE = bits worden gewist.
	BaseType_t waitForAllbits = pdTRUE;			// FALSE = wacht totdat één v/d bits is gezet, TRUE wacht op ALLE bits.
	
	TickType_t ticksToWait	  = portMAX_DELAY;	// Maximale wachttijd, portMAX_DELAY = onbeperkt wachten.


	bool homingDone = false;
	bool cycleDone  = false;

	uint32_t flags = 0;
	
	vPrintString("> starting ControlTask.\n");

	// schakeld direct de motoren uit.
	motor_DisableESCONController();

	// setup external 1 ms timer tick handler:
	// maak Semaphore aan
	uint32_t maxSemCount = 1;
	uint32_t initialSemCount = 0;
	TimerInterruptSemaphore = xSemaphoreCreateCounting(maxSemCount, initialSemCount);
	handle_EmergencySemaphore = xSemaphoreCreateCounting(maxSemCount, initialSemCount);

	
	// Bij opkomend signaal in PIN, run ClockInterruptHandler.
	flags = PIO_IT_RISE_EDGE;
	interrupt_AttachHandler(ClockInterruptHandler, CLOCK_PIN, flags);
	
	// Bij opkomend signaal in PINs, run EmergencyInterruptHandler.
	flags = PIO_IT_RISE_EDGE;
	interrupt_AttachHandler(EmergencyInterruptHandler, NOODSTOP_PIN, flags);
	interrupt_AttachHandler(EmergencyInterruptHandler, NOODSCHAKEL_PIN, flags);
	interrupt_AttachHandler(EmergencyInterruptHandler, ESCONFOUT_PIN, flags);


	vPrintString("> ControlTask waiting for helper tasks...\n");

	///////////////////////////////////////////////////////////////////////////
	// startup indicatie
	// Rood en oranje samen knipperen zolang helper tasks nog niet ready zijn


	// wait for all tasks to get up and running:
	threadBits = xEventGroupWaitBits(handle_ThreadEventGroup, BIT_0 | BIT_1, stayAllbits, waitForAllbits, ticksToWait);
	vPrintString("> helper tasks running, ControlTask started, event group = 0x%04x\n", threadBits);
	state =  STATE_FAULT;
	
	///////////////////////////////////////////////////////////////////////////
	// hoofd-loop
	while (true)
	{
		// Lees knopbits of er iets ingedrukt is.
		buttonBits = xEventGroupWaitBits(handle_ButtonEventGroup,
			BIT_START_BUTTON | BIT_STOP_BUTTON | BIT_RESET_BUTTON,
			clearAllbits,		// ontvangen bits wissen
			dontWaitForAllbits,	// één van de bits is genoeg
			0					// niet blokkeren
		);

		// Emergency check via Semaphore
		if (xSemaphoreTake(handle_EmergencySemaphore, 0) == pdTRUE)
		{
			vPrintString("> EMERGENCY INTERRUPT RECIEVED\n");
			state =  STATE_FAULT;
		}

		switch (state)
		{
			/////////////////////////////////////////////////////////////////////
			case  STATE_WAIT:
			{
				// Wachten op start of reset -> homing starten
				if ((buttonBits & BIT_START_BUTTON) || (buttonBits & BIT_RESET_BUTTON))
				{
					vPrintString("> WAIT -> HOMING ( Start-of Resetknop is ontvangen).\n");
					homingDone = false;
					state =  STATE_HOMING;
				}

				taskSleep(10);
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_HOMING:
			{

				//TODO: homingDone = HomingStep(); <- MotorControl.c

				if (homingDone)
				{
					vPrintString("> HOMING complete -> READY\n");
					state =  STATE_READY;
				}
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_READY:
			{
				// Op vaste positie regelen op iedere control tick
				xSemaphoreTake(TimerInterruptSemaphore, ticksToWait);
				
				//TODO: HoldPosition()

				// Startknop -> runnen
				if (buttonBits & BIT_START_BUTTON)
				{
					vPrintString("> READY -> RUNNING (Startknop ontvangen.)\n");
					cycleDone = false;
					state = STATE_RUNNING;
				}

				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_RUNNING:
			{
				// Sequence draaien op control tick
				xSemaphoreTake(TimerInterruptSemaphore, ticksToWait);
	
				//TODO: cycleDone = RunSequenceStep();


				// Stopknop -> terug naar READY
				if (buttonBits & BIT_STOP_BUTTON)
				{
					vPrintString("> RUNNING -> READY (stopknop ontvangen).\n");
					state = STATE_READY;
				}
				// Cyclus klaar -> terug naar READY
				else if (cycleDone)
				{
					vPrintString("> RUNNING -> READY (cyclus voldaan)\n");
					state = STATE_READY;
				}

				break;
			}

			/////////////////////////////////////////////////////////////////////
			case  STATE_FAULT:
			{
				// Veilig maken
				motor_DisableESCONController();

				taskSleep(10);

				// Alleen uit fault als reset is gedrukt EN foutsignaal weg is
				if (buttonBits & BIT_RESET_BUTTON)
				{
					vPrintString("> FAULT cleared -> WAIT (reset ontvangen)\n");
					state = STATE_WAIT;
				}
				break;
			}

			/////////////////////////////////////////////////////////////////////
			default: //Onbekende Status?
			{
				state = STATE_FAULT;
				break;
			}
		}
	}

	/* Should never get here */
}


///////////////////////////////////////////////////////////////////////////////
// void ControlTask(void *pvParameters)
/*
void ControlTask(void *pvParameters)
{
	uint32_t flags = 0;
	uint32_t maxSemCount = 1;
	uint32_t initialSemCount = 0;

	EventBits_t uxBits = 0;
	BaseType_t clearAllbits	  = pdFALSE;		// FALSE = bits blijven staan na continue, TRUE = bits worden gewist.
	BaseType_t waitForAllbits = pdTRUE;			// FALSE = wacht totdat één v/d bits is gezet, TRUE wacht op ALLE bits.
	TickType_t ticksToWait	  = portMAX_DELAY;	// Maximale wachttijd, portMAX_DELAY = onbeperkt wachten.
	
	double wblFactor = 0.0;
	
	vPrintString("> starting ControlTask (load)\n");

	// schakeld direct de motoren uit.
	motor_DisableESCONController();

	// setup external 1 ms timer tick handler:
	// maak Semaphore aan
	TimerInterruptSemaphore = xSemaphoreCreateCounting(maxSemCount, initialSemCount);
	
	// Bij opkomend signaal in PIN, run ClockInterruptHandler. 
	flags = PIO_IT_RISE_EDGE;
	interrupt_AttachHandler(ClockInterruptHandler, PIN_30, flags);
	
	vPrintString("> ControlTask waiting for helper tasks...\n");

	// wait for ButtonHandlerTask and ParameterSettingTask to get up and running:
	// Wacht tot BIT_1 AND BIT_0 TRUE zijn in handle_ThreadEventGroup, 
	uxBits = xEventGroupWaitBits(handle_ThreadEventGroup, BIT_1 | BIT_0,
								 clearAllbits, waitForAllbits, ticksToWait);	

	vPrintString("> helper tasks running, ControlTask started, event group = 0x%04x\n", uxBits);
	
	// Stel de Quadraturecounters in.
	QCEncodersSetup();
	
	// Schakelt de Escon in en stuurt naar home positie.
	motor_EnableESCONController(); 
	motor_GotoHomePosition(MOVE_LEFT); 

	// zet Quadraturecounter op nul en print dit.
	QCEncodersClearCount();
	QCEncodersShowCount("> Initial home");

	vPrintString("> ready\n");
	vPrintString("> press button SW1 to start (watch your fingers...)\n");
	
	// Wacht tot reset/start knop ingedrukt wordt.
	ticksToWait = portMAX_DELAY;
	xSemaphoreTake(handle_RestartSemaphore, ticksToWait);	// wait for SW1 first button press
	
	while (true)
	{
		// Zoek nogmaals de home positie op, zet de QC op nul en print dit.
		motor_GotoHomePosition(MOVE_LEFT);
		QCEncodersClearCount();
		QCEncodersShowCount("> HOME");
		
		// Check ingestelde bandbreedte waarde en print.
		// always leave parameter value in queue! So use xQueuePeek
		ticksToWait = 0;
		xQueuePeek(handle_ParameterQueue, &wblFactor, ticksToWait);
		vPrintString("> running with wblFactor: %.3f\n", wblFactor);
		
		// Mechanische parameters voor regeling bepalen
		posctrlLoad_InitParameters(wblFactor);
		
		
		ControlLoop();	// this loop exits by pressing button SW1
	}
	
	/* Should never go here */
	vTaskDelete(NULL);
}


///////////////////////////////////////////////////////////////////////////////
//  void ControlLoop(void)

void ControlLoop(void)
{
	vPrintString("> enter Control Loop (load)\n");

	BaseType_t restart = pdFALSE;
	BaseType_t ticksToWait = 0;
	bool continueControlLoop = true;
	
	
	while (continueControlLoop)
	{
		// wait for periodic 1 ms timer tick to unblock this thread and
		// run the motion controller:
		xSemaphoreTake(TimerInterruptSemaphore, portMAX_DELAY);
		posctrlLoad_RunController();
		
		// check restart semaphore here, invoked by button press
		restart = xSemaphoreTake(handle_RestartSemaphore, ticksToWait);
		if (restart == pdTRUE)
		{
			continueControlLoop = false;
		}
		taskSleep(0);
	}

	vPrintString("> exit Control Loop (load)\n");
}
*/