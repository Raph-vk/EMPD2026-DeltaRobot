/*
 * ApplicationTasks.c
 *
 * Created: 27-11-2023 14:57:44
 *  Author: raphv
 
Deze file is de centrale opstart van de applicatie.
 
 */
#include "ApplicationTasks.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "vPrintString.h"

#include "ControlTask.h"
#include "InputHandlerTask.h"
#include "VisualisationTask.h"
#include "DisturbanceCompensation.h"

///////////////////////////////////////////////////////////////////////////////
// application tasks handler declarations

static TaskHandle_t handle_InputHandlerTask = NULL;
static TaskHandle_t handle_VisualisationTask = NULL;
TaskHandle_t		handle_ControlTask = NULL;

///////////////////////////////////////////////////////////////////////////////
// global handles & objects

EventGroupHandle_t handle_ThreadEventGroup = NULL;
EventGroupHandle_t handle_ButtonEventGroup = NULL;

SemaphoreHandle_t	handle_NoodSemaphore = NULL;

QueueHandle_t		handle_StateQueue = NULL;
QueueHandle_t		handle_potQueue = NULL;
QueueHandle_t		handle_stroomQueue = NULL;
QueueHandle_t		handle_DisturbanceQueue = NULL;


///////////////////////////////////////////////////////////////////////////////
// void StartApplicationTasks(void)
/*
 * Initialiseert de FreeRTOS eventgroups, semaphores en queues.
 * Maakt daarna de applicatietaken aan en bewaart de task-handles.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; de globale handles worden gevuld.
 */
void StartApplicationTasks(void)
{
	BaseType_t result = pdFAIL;
	UBaseType_t QueueSize = 1;

	// Event group aanmaken voor knoppen, communicatie tussen InputHandlerTask -> ControlTask.
	handle_ThreadEventGroup = xEventGroupCreate();
	if (handle_ThreadEventGroup == NULL)
	{
		vPrintString("handle_ThreadEventGroup create failed.\n");
	}
	
	
	// Event group aanmaken, Deze wordt gebruikt om te wachten tot alle taken klaar zijn met opstarten.
	// Wanneer parameter setting taak & button handler taak zijn opgestart wordt controltaak pas opgestart.
	//// event group to wait for parameter setting task & button handler task before
	//// running the control task on startup
	handle_ButtonEventGroup = xEventGroupCreate();
	if (handle_ButtonEventGroup == NULL)
	{
		vPrintString("handle_ButtonEventGroup create failed.\n");
	}
	
	// NoodSmephore aanmaken
	handle_NoodSemaphore = xSemaphoreCreateBinary();
	if (handle_NoodSemaphore == NULL)
	{
		vPrintString("handle_NoodSemaphore create failed.\n");
	}

	// Aanmaken van "SystemState queue"
	// Geeft de actuele status waarin de machine zich bevindt door.
	handle_StateQueue = xQueueCreate(QueueSize, sizeof(SystemState_t));
	if (handle_StateQueue == NULL)
	{
		vPrintString("handle_StateQueue create failed.\n");
	}
	

	// Aanmaken van "potQueue"
	// Geeft de actuele procentuele stap-waarde van de potmeter door.
	handle_potQueue = xQueueCreate(QueueSize, sizeof(uint32_t));
	if (handle_potQueue == NULL)
	{
		vPrintString("handle_potQueue create failed.\n");
	}

	// Aanmaken van "stroom queue"
	// Geeft de actuele stroomwaarde door.
	handle_stroomQueue = xQueueCreate(QueueSize, sizeof(float));
	if (handle_stroomQueue == NULL)
	{
		vPrintString("handle_stroomQueue create failed.\n");
	}

	// Aanmaken van "Disturbance queue"
	// Geeft de actuele X- en Y-verstoring van het frame door.
	handle_DisturbanceQueue = xQueueCreate(QueueSize, sizeof(DisturbanceMeasurement_t));

	if (handle_DisturbanceQueue == NULL)
	{
		vPrintString("handle_DisturbanceQueue create failed.\n");
	}
	
	/**************************************************** Taken aanmaken ****************************************************/
	// De taken worden aangemaakt en in de ready list geplaatst.
	// Prioriteit 0 is laagste 4 is hoogste!!
	result = xTaskCreate(InputHandlerTask, "tsk_Input", (configMINIMAL_STACK_SIZE), NULL, 0, &handle_InputHandlerTask);
	if (result != pdPASS )
	{
		vPrintString("InputHandlerTask task create failed.\n");
	}
	result = xTaskCreate(ControlTask, "tsk_Control", (configMINIMAL_STACK_SIZE), NULL, 2, &handle_ControlTask);
	if (result != pdPASS )
	{
		vPrintString("ControlTask task create failed.\n");
	}
	result = xTaskCreate(VisualisationTask, "tsk_Visualisation", (configMINIMAL_STACK_SIZE), NULL, 1, &handle_VisualisationTask);
	if (result != pdPASS )
	{
		vPrintString("VisualisationTask task create failed.\n");

	}
	/**************************************************** Taken aanmaken ****************************************************/
}

