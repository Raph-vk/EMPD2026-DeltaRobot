/*
 * InputHandlerTask.c
 *
 * Created: 09-04-2026
 *  Author: Raph van Koeveringe
 */ 
///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"


///////////////////////////////////////////////////////////////////////////////
// other includes

#include "SwitchLib.h"
#include "bits.h"
#include "InputHandlerTask.h"
#include "ApplicationTasks.h"
#include "MotorControl.h" 


///////////////////////////////////////////////////////////////////////////////
// void Inturrupt(void *pvParameters)
void NoodInterruptHandler(uint32_t id, uint32_t mask)
{
	motor_DisableESCONController(); //Direct motoren uitschakelen
	
	// Als interrupt Semaphore is aangemaakt, geef vrij.
	if (handle_NoodSemaphore != NULL)
	{
		xSemaphoreGiveFromISR(handle_NoodSemaphore, NULL);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void InputHandlerTask(void *pvParameters)

//Deze taak bewaakt de knoppen en comminuceert dat.
void InputHandlerTask(void *pvParameters)
{
	uint8_t buttonStart = 0;	// 0 == SW1 (start)
	uint8_t buttonStop = 1;		// 1 == SW2 (restart)
	uint8_t buttonRestart = 2;	// 2 == SW3 (stop)
	
	vPrintString("> starting InputHandlerTask\n");

	// Laat weten aan de control task dat deze helper-task actief is, via een eventBit.
	// signal to control thread that InputHandlerTask is up and running:
	xEventGroupSetBits( handle_ThreadEventGroup, BIT_0 );
	
	while(true)
	{
		// Wanneer knop wordt ingedrukt
		if (switch_IsPressed(buttonStart))
		{
			// wait until button released:
			while (switch_IsPressed(buttonStart))
			{
			}
			
			// Laat weten dat knop is indrukt
			vPrintString("> Start button SW%d pressed!\n", buttonStart);
			// ControlTask -> ControlLoop neemt Semaphore.
			xSemaphoreGive(handle_StartSemaphore);
		}
		// Wanneer knop wordt ingedrukt
		else if (switch_IsPressed(buttonStop))
		{
			// wait until button released:
			while (switch_IsPressed(buttonStop))
			{
			}
			
			// Laat weten dat knop is indrukt
			vPrintString("> Stop button SW%d pressed!\n", buttonStop);

			// ControlTask -> ControlLoop neemt Semaphore.
			xSemaphoreGive(handle_StopSemaphore);
		}
		else if (switch_IsPressed(buttonReset))
		{
			// wait until button released:
			while (switch_IsPressed(buttonReset))
			{
			}
			
			// Laat weten dat knop is indrukt
			vPrintString("> Start button SW%d pressed!\n", buttonReset);
			// ControlTask -> ControlLoop neemt Semaphore.
			xSemaphoreGive(handle_ResetSemaphore);
		}
		else
		{
			taskSleep(10);
		}
	}
}