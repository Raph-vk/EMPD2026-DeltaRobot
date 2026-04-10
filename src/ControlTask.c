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


///////////////////////////////////////////////////////////////////////////////
// void ControlTask(void *pvParameters)

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
