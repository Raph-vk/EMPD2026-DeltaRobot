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
#include "VisualisationTask.h"
#include "ControlTask.h"


///////////////////////////////////////////////////////////////////////////////
// application tasks handler declarations

static TaskHandle_t handle_ButtonHandlerTask = NULL;
static TaskHandle_t handle_ControlTask = NULL;
static TaskHandle_t handle_VisualisationTask = NULL;


///////////////////////////////////////////////////////////////////////////////
// global handles & objects

EventGroupHandle_t handle_ThreadEventGroup = NULL;
EventGroupHandle_t handle_ButtonEventGroup = NULL;

SemaphoreHandle_t  handle_TimerInterruptSemaphore = NULL;
SemaphoreHandle_t  handle_EmergencySemaphore = NULL;

QueueHandle_t      handle_StateQueue = NULL;



///////////////////////////////////////////////////////////////////////////////
// void StartApplicationTasks(void)
void StartApplicationTasks(void)
{
	BaseType_t result = pdFAIL;
	UBaseType_t StateQueueSize = 1;
	UBaseType_t maxSemCount = 1;
	UBaseType_t initialSemCount = 0;

	// Aanmaken van "SystemState queue"
	// Geeft de actuele status waarin de machine zich bevindt door.
	handle_StateQueue = xQueueCreate(StateQueueSize, sizeof(SystemState_t));
	if (handle_StateQueue == NULL)
	{
		//Foutafhandeling is leeggelaten
	}
	
	// Semaphore voor externe control-ticks.
	handle_TimerInterruptSemaphore = xSemaphoreCreateCounting(maxSemCount, initialSemCount);
	if (handle_TimerInterruptSemaphore == NULL)
	{
		// Foutafhandeling is leeggelaten
	}
	
	// EmergencySmephore aanmaken
	handle_EmergencySemaphore = xSemaphoreCreateBinary();
	if (handle_EmergencySemaphore == NULL)
	{
		// Foutafhandeling is leeggelaten
	}
	
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
	

	

	
	/**************************************************** Taken aanmaken ****************************************************/
	// De taken worden aangemaakt en in de ready list geplaatst.
	// Prioriteit 0 is laagste 4 is hoogste!!
	result = xTaskCreate(ButtonHandlerTask, "tsk_Button", (configMINIMAL_STACK_SIZE), NULL, 0, &handle_ButtonHandlerTask);
	if (result == pdPASS )
	{
	}
	result = xTaskCreate(ControlTask, "tsk_Control", (configMINIMAL_STACK_SIZE), NULL, 2, &handle_ControlTask);
	if (result == pdPASS )
	{
	}
	result = xTaskCreate(VisualisationTask, "tsk_Visualisation", (configMINIMAL_STACK_SIZE), NULL, 1, &handle_VisualisationTask);
	if (result == pdPASS )
	{
	}

	/**************************************************** Taken aanmaken ****************************************************/
}

