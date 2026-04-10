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

///////////////////////////////////////////////////////////////////////////////
// global handles & objects

EventGroupHandle_t	handle_ThreadEventGroup = NULL;
EventGroupHandle_t	handle_ButtonEventGroup = NULL;

SemaphoreHandle_t	TimerInterruptSemaphore = NULL;
SemaphoreHandle_t	handle_EmergencySemaphore = NULL;

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
	
	// Event group aanmaken voor knoppen, communicatie tussen ButtonHandlerTask -> ControlTask.
	handle_ThreadEventGroup = xEventGroupCreate();
	if (handle_ThreadEventGroup == NULL)
	{
		// Foutafhandeling is leeggelaten
	}
	
	// Event group aanmaken, Deze wordt gebruikt om te wachten tot alle taken klaar zijn met opstarten.
	// Wanneer parameter setting taak & button handler taak zijn opgestart wordt controltaak pas opgestart.
	//// event group to wait for parameter setting task & button handler task before
	//// running the control task on startup
	handle_ButtonEventGroup = xEventGroupCreate();
	if (handle_ButtonEventGroup == NULL)
	{
		// Foutafhandeling is leeggelaten
	}
	
	// EmergencySmephore aanmaken
	handle_EmergencySemaphore = xSemaphoreCreateBinary();
	if (handle_EmergencySemaphore == NULL)
	{
		// Foutafhandeling is leeggelaten
	}	
	

	
	/**************************************************** Taken aanmaken ****************************************************/
	// De taken worden aangemaakt en in de ready list geplaatst.
	result = xTaskCreate(ButtonHandlerTask, "tsk_Input", (configMINIMAL_STACK_SIZE), NULL, 0, &handle_ButtonHandlerTask);
	if (result == pdPASS )
	{
	}
	result = xTaskCreate(ControlTask, "tsk_Control", (configMINIMAL_STACK_SIZE), NULL, 1, &handle_ControlTask);
	if (result == pdPASS )
	{
	}

	/**************************************************** Taken aanmaken ****************************************************/
}

