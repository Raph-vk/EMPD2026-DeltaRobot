/*
 * ButtonHandlerTask.c
 *
 * Created: 23-11-2023 13:11:18
 *  Author: rasmsmee
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
/* void ButtonHandlerTask(void *pvParameters)

Deze taak bewaakt knop SW1 en gebruikt die als restart/start-knop voor de regeling. 

*/
void ButtonHandlerTask(void *pvParameters)
{
	uint8_t buttonNumber = 0;	// 0 == SW1
	
	vPrintString("> starting ButtonHandlerTask\n");

	// Laat weten aan de control task dat deze helper-task actief is, via een eventBit.
	// signal to control thread that ButtonHandlerTask is up and running:
	xEventGroupSetBits( handle_ThreadEventGroup, BIT_0 );
	
	while(true)
	{
		// Wanneer knop wordt ingedrukt
		if (switch_IsPressed(buttonNumber))
		{
			// wait until button released:
			while (switch_IsPressed(buttonNumber))
			{
			}
			
			// Laat weten dat knop is indrukt
			vPrintString("> restart button SW%d pressed!\n", buttonNumber + 1);
			
			// ControlTask -> ControlLoop neemt Semaphore.
			xSemaphoreGive(handle_RestartSemaphore);
		}
		taskSleep(10);
	}
}
