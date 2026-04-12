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
//#include "bits.h"
//#include "PortIOLib.h"
#include "MachinePins.h"

#include "ApplicationTasks.h"

///////////////////////////////////////////////////////////////////////////////
/* void ButtonHandlerTask(void *pvParameters)

Bewaakt de knoppen Start, Stop en Reset en zet eventbits voor de ControlTask.

*/
void ButtonHandlerTask(void *pvParameters)
{
	uint8_t startButton = PCB_SWITCH_START;   // SW1
	uint8_t stopButton  = PCB_SWITCH_STOP;   // SW2
	uint8_t resetButton = PCB_SWITCH_RESET;   // SW3

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
				xEventGroupSetBits(handle_ButtonEventGroup, EVT_START_BUTTON);
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
				xEventGroupSetBits(handle_ButtonEventGroup, EVT_STOP_BUTTON);
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
				xEventGroupSetBits(handle_ButtonEventGroup, EVT_RESET_BUTTON);
			}
		}

		taskSleep(10);
	}
}