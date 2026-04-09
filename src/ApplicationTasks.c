/*
 * ApplicationTasks.c
 *
 * Created: 27-11-2023 14:57:44
 *  Author: rasmsmee
 
Deze file is de centrale opstart van de applicatie.
 
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

#include "ApplicationTasks.h"
#include "ButtonHandlerTask.h"
#include "ParameterSettingTask.h"
#include "ControlTask.h"


///////////////////////////////////////////////////////////////////////////////
// application tasks handler declarations

static TaskHandle_t handle_InputHandlerTask	= NULL;
static TaskHandle_t handle_ContolTask		= NULL;
static TaskHandle_t handle_MotionControlTask= NULL;

///////////////////////////////////////////////////////////////////////////////
// global handles & objects

EventGroupHandle_t	handle_ThreadEventGroup = NULL;
SemaphoreHandle_t	handle_StartSemaphore = NULL;
SemaphoreHandle_t	handle_StopSemaphore = NULL;
SemaphoreHandle_t	handle_ResetSemaphore = NULL;
SemaphoreHandle_t	TimerInterruptSemaphore = NULL;
SemaphoreHandle_t	handle_NoodSemaphore = NULL;

//QueueHandle_t		handle_ParameterQueue	= NULL;


///////////////////////////////////////////////////////////////////////////////
// void StartApplicationTasks(void)
void StartApplicationTasks(void)
{
	BaseType_t result = pdFAIL;
	UBaseType_t parameterQueueSize = 1;

	// Aanmaken van "parameter queue"
	// Bandbreedte is aanpasbaar met potmeter, die ingestelde waarde wordt hier doorgegeven.
	// queue for bandwidth parameter passing
	//handle_InputQueue = xQueueCreate(parameterQueueSize, sizeof(double));
	//if (handle_ParameterQueue == NULL)
	//{
		//Foutafhandeling is leeggelaten
	//}
	
	// Aanmaken van Semaphore voor Knoppen, communicatie tussen InputHandlerTask -> ControlTask.

	// restart button semaphore
	handle_NoodSemaphore = xSemaphoreCreateBinary();
		if (handle_noodSemaphore == NULL)
		{
			// Foutafhandeling is leeggelaten
		}
	handle_StartSemaphore = xSemaphoreCreateBinary();
		if (handle_StartSemaphore == NULL)
		{
			// Foutafhandeling is leeggelaten
		}
	// restart button semaphore
	handle_StopSemaphore = xSemaphoreCreateBinary();
		if (handle_StopSemaphore == NULL)
		{
			// Foutafhandeling is leeggelaten
		}
	// restart button semaphore
	handle_ResetSemaphore = xSemaphoreCreateBinary();
		if (handle_ResetSemaphore == NULL)
		{
			// Foutafhandeling is leeggelaten
		}
	

	// Event group aanmaken, Deze wordt gebruikt om te wachten tot alle taken klaar zijn met opstarten.
	// Wanneer parameter setting taak & button handler taak zijn opgestart wordt controltaak pas opgestart.
	//// event group to wait for parameter setting task & button handler task before
	//// running the control task on startup
	handle_ThreadEventGroup = xEventGroupCreate();
	if (handle_ThreadEventGroup == NULL)
	{
		// Foutafhandeling is leeggelaten
	}
	
	/**************************************************** Taken aanmaken ****************************************************/
	// De taken worden aangemaakt en in de ready list geplaatst.
	result = xTaskCreate(InputHandlerTask, "tsk_Input", (configMINIMAL_STACK_SIZE), NULL, 0, &handle_InputHandlerTask);
	if (result == pdPASS )
	{
	}
	result = xTaskCreate(ControlTask, "tsk_Control", (configMINIMAL_STACK_SIZE), NULL, 1, &handle_ControlTask);
	if (result == pdPASS )
	{
	}

	result = xTaskCreate(MotionControlTask, "tsk_Motion", (configMINIMAL_STACK_SIZE), NULL, 4, &handle_MotionControlTask);
	if (result == pdPASS )
	{
	}
	/**************************************************** Taken aanmaken ****************************************************/
}

