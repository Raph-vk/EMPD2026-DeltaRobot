/*
 * ButtonHandlerTask.c
 *
 * Created: 10/04/2023
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
#include "ButtonHandlerTask.h"
#include "bits.h"
#include "ApplicationTasks.h"


///////////////////////////////////////////////////////////////////////////////
// void EmergencyHandler(uint32_t id, uint32_t mask)
//
// invoked on every clock tick (1 ms) of the external hardware clock
void EmergencyInterruptHandler(uint32_t id, uint32_t mask)
{
	motor_DisableESCONController();
	
	// Als EmergencySemaphore is aangemaakt, geef vrij.
	if (handle_EmergencySemaphore != NULL)
	{
		xSemaphoreGiveFromISR(handle_EmergencySemaphore, NULL);
	}
	
}

///////////////////////////////////////////////////////////////////////////////
/* void ButtonHandlerTask(void *pvParameters)

Bewaakt de knoppen Start, Stop en Reset en zet eventbits voor de ControlTask.

*/
void ButtonHandlerTask(void *pvParameters)
{
	uint8_t startButton = 0;   // SW1
	uint8_t stopButton  = 1;   // SW2
	uint8_t resetButton = 2;   // SW3

	vPrintString("> starting ButtonHandlerTask\n");

	// Apart aangeven dat deze task actief is
	xEventGroupSetBits(handle_ThreadEventGroup, BIT_0);

	while (true)
	{
		// START
		if (switch_IsPressed(startButton))
		{
			taskSleep(20); // debounce
			if (switch_IsPressed(startButton))
			{
				while (switch_IsPressed(startButton))
				{
					taskSleep(1);
				}

				vPrintString("> START button SW%d pressed!\n", startButton + 1);
				xEventGroupSetBits(handle_ButtonEventGroup, BIT_START_BUTTON);
			}
		}

		// STOP
		if (switch_IsPressed(stopButton))
		{
			taskSleep(20); // debounce
			if (switch_IsPressed(stopButton))
			{
				while (switch_IsPressed(stopButton))
				{
					taskSleep(1);
				}

				vPrintString("> STOP button SW%d pressed!\n", stopButton + 1);
				xEventGroupSetBits(handle_ButtonEventGroup, BIT_STOP_BUTTON);
			}
		}

		// RESET
		if (switch_IsPressed(resetButton))
		{
			taskSleep(20); // debounce
			if (switch_IsPressed(resetButton))
			{
				while (switch_IsPressed(resetButton))
				{
					taskSleep(1);
				}

				vPrintString("> RESET button SW%d pressed!\n", resetButton + 1);
				xEventGroupSetBits(handle_ButtonEventGroup, BIT_RESET_BUTTON);
			}
		}

		taskSleep(10);
	}
}