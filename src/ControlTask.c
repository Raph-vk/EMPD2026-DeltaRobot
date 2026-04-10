/*
 * ControlTask.c
 *
 * Created: 10-9-2023 09:48:15
 *  Author: rasmsmee
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DeviceIOLib.h"
#include "InterruptLib.h"
#include "bits.h"

///////////////////////////////////////////////////////////////////////////////
// position controller & application includes

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
	// Als interrupt Semaphore is aangemaakt, geef vrij.
	if (TimerInterruptSemaphore != NULL)
	{
		xSemaphoreGiveFromISR(TimerInterruptSemaphore, NULL);
	}
}

/*
 * ControlTask.c
 *
 * Created: 10-9-2023 09:48:15
 * Author: rasmsmee / uitgebreid door Raph
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
// local type definitions
typedef enum
{
	CONTROL_STATE_INIT = 0,
	CONTROL_STATE_WAIT,
	CONTROL_STATE_HOMING,
	CONTROL_STATE_READY,
	CONTROL_STATE_RUNNING,
	CONTROL_STATE_FAULT
} ControlState_t;

///////////////////////////////////////////////////////////////////////////////
// local function prototypes

static void SetLedsWait(void);
static void SetLedsHoming(void);
static void SetLedsReady(void);
static void SetLedsRunning(void);
static void SetLedsFault(void);

static void InitMechanicalParameters(void);

static bool HomingStep(void);
static void HoldOnPosStep(void);
static bool RunSequenceStep(void);

static bool IsEmergencyActive(void);

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
	ControlState_t state = CONTROL_STATE_INIT;
	
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

	while ((xEventGroupGetBits(handle_ThreadEventGroup) & (BIT_0 | BIT_1)) != (BIT_0 | BIT_1))
	{
		// Oranje aan
		port_SetLamp(LAMP_ORANGE);
		taskSleep(200);

		// Rood aan
		port_SetLamp(LAMP_RED);
		taskSleep(200);
	}

	port_AllLampsOff();

	// wait for all tasks to get up and running:
	threadBits = xEventGroupWaitBits(handle_ThreadEventGroup, BIT_0 | BIT_1, stayAllbits, waitForAllbits, ticksToWait);

	vPrintString("> helper tasks running, ControlTask started, event group = 0x%04x\n", threadBits);

	///////////////////////////////////////////////////////////////////////////
	// hoofdloop

	while (true)
	{
		// Lees knopbits niet-blockerend uit
		buttonBits = xEventGroupWaitBits(handle_ButtonEventGroup,
			BIT_START_BUTTON | BIT_STOP_BUTTON | BIT_RESET_BUTTON,
			clearAllbits,		// ontvangen bits wissen
			dontWaitForAllbits,	// één van de bits is genoeg
			0					// niet blokkeren
		);

		// Emergency check via Semaphore
		if (xSemaphoreTake(handle_EmergencySemaphore, 0) == pdTRUE)
		{
			vPrintString("> emergency interrupt received\n");
			state = CONTROL_STATE_FAULT;
		}

		switch (state)
		{
			/////////////////////////////////////////////////////////////////////
			case CONTROL_STATE_WAIT:
			{
				SetLedsWait();

				// Wachten op start of reset -> homing starten
				if ((buttonBits & BIT_START_BUTTON) || (buttonBits & BIT_RESET_BUTTON))
				{
					vPrintString("> WAIT -> HOMING\n");
					homingDone = false;
					state = CONTROL_STATE_HOMING;
				}

				taskSleep(10);
				break;
			}

			/////////////////////////////////////////////////////////////////////
			case CONTROL_STATE_HOMING:
			{
				SetLedsHoming();

				// Homing draait op 1 ms control tick
				if (xSemaphoreTake(TimerInterruptSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
				{
					homingDone = HomingStep();

					if (homingDone)
					{
						vPrintString("> HOMING complete -> READY\n");
						state = CONTROL_STATE_READY;
					}
				}

				break;
			}

			/////////////////////////////////////////////////////////////////////
			case CONTROL_STATE_READY:
			{
				SetLedsReady();

				// Positie vasthouden op control tick
				if (xSemaphoreTake(TimerInterruptSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
				{
					HoldOnPosStep();
				}

				// Startknop -> runnen
				if (buttonBits & BIT_START_BUTTON)
				{
					vPrintString("> READY -> RUNNING\n");
					cycleDone = false;
					state = CONTROL_STATE_RUNNING;
				}

				break;
			}

			/////////////////////////////////////////////////////////////////////
			case CONTROL_STATE_RUNNING:
			{
				SetLedsRunning();

				// Sequence draaien op control tick
				if (xSemaphoreTake(TimerInterruptSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
				{
					cycleDone = RunSequenceStep();
				}

				// Stopknop -> terug naar READY
				if (buttonBits & BIT_STOP_BUTTON)
				{
					vPrintString("> RUNNING -> READY (stop button)\n");
					state = CONTROL_STATE_READY;
				}
				// Cyclus klaar -> terug naar READY
				else if (cycleDone)
				{
					vPrintString("> RUNNING -> READY (cycle done)\n");
					state = CONTROL_STATE_READY;
				}

				break;
			}

			/////////////////////////////////////////////////////////////////////
			case CONTROL_STATE_FAULT:
			{
				SetLedsFault();

				// Veilig maken
				motor_DisableESCONController();

				taskSleep(10);

				// Alleen uit fault als reset is gedrukt EN foutsignaal weg is
				if (buttonBits & BIT_RESET_BUTTON)
				{
					if (!IsEmergencyActive())
					{
						vPrintString("> FAULT cleared -> WAIT\n");
						state = CONTROL_STATE_WAIT;
					}
				}

				break;
			}

			/////////////////////////////////////////////////////////////////////
			default:
			{
				state = CONTROL_STATE_FAULT;
				break;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// void port_AllLampsOff(void)

void port_AllLampsOff(void)
{
	port_SetBit(LAMP_GREEN, false);
	port_SetBit(LAMP_ORANGE, false);
	port_SetBit(LAMP_RED, false);
}

///////////////////////////////////////////////////////////////////////////////
// void port_SetLamp(uint8_t lampNumber)
// Zet een lamp aan
// 
void port_SetLamp(uint8_t lampNumber)
{
	port_AllLampsOff();

	if (lampNumber == LAMP_GREEN)
	{
		port_SetBit(LAMP_GREEN, true);
	}
	else if (lampNumber == LAMP_ORANGE)
	{
		port_SetBit(LAMP_ORANGE, true);
	}
	else if (lampNumber == LAMP_RED)
	{
		port_SetBit(LAMP_RED, true);
	}
	else
	{
		// ongeldige keuze -> alle lampen uit
	}
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