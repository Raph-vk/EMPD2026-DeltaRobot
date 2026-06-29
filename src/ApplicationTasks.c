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
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "vPrintString.h"

#include "ControlTask.h"
#include "InputHandlerTask.h"
#include "VisualisationTask.h"

///////////////////////////////////////////////////////////////////////////////
// defines
#define STACK_INPUT_TASK          (configMINIMAL_STACK_SIZE * 2)
#define STACK_CONTROL_TASK        (configMINIMAL_STACK_SIZE * 5)
#define STACK_VISUALISATION_TASK  (configMINIMAL_STACK_SIZE * 3)

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
SemaphoreHandle_t	handle_OffsetZeroRequest = NULL;
SemaphoreHandle_t	handle_OffsetZeroDone = NULL;

QueueHandle_t		handle_StateQueue = NULL;
QueueHandle_t		handle_stroomQueue = NULL;
QueueHandle_t		handle_OffsetQueue = NULL;
QueueHandle_t		handle_DisplayInfoQueue = NULL;
//QueueHandle_t		handle_DisturbanceQueue = NULL;

static volatile bool tcpCompEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// void DisplayInfo_Publish(const char *regel1, const char *regel2)
/*
 * Publiceert twee algemene displayregels naar de VisualisationTask.
 * Invoer: regel1 en regel2 zijn nul-getermineerde tekstregels.
 * Uitvoer: geen returnwaarde; overschrijft de laatste display-info in de queue.
 */
void DisplayInfo_Publish(const char *regel1, const char *regel2)
{
	if ((handle_DisplayInfoQueue != NULL) && (regel1 != NULL) && (regel2 != NULL))
	{
		DisplayInfo_t displayInfo;

		snprintf(displayInfo.regel1, sizeof(displayInfo.regel1), "%s", regel1);
		snprintf(displayInfo.regel2, sizeof(displayInfo.regel2), "%s", regel2);

		xQueueOverwrite(handle_DisplayInfoQueue, &displayInfo);
	}
}

bool TcpCompensation_IsEnabled(void)
{
	return tcpCompEnabled;
}

void TcpCompensation_SetEnabled(bool enabled)
{
	tcpCompEnabled = enabled;
}

bool TcpCompensation_Toggle(void)
{
	tcpCompEnabled = !tcpCompEnabled;
	return tcpCompEnabled;
}


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

	// Semaphore waarmee Homing/InputHandler een offset-nulmeting kan aanvragen.
	handle_OffsetZeroRequest = xSemaphoreCreateBinary();
	if (handle_OffsetZeroRequest == NULL)
	{
		vPrintString("handle_OffsetZeroRequest create failed.\n");
	}

	// Semaphore waarmee InputHandler meldt dat offset-nulmeting klaar is.
	handle_OffsetZeroDone = xSemaphoreCreateBinary();
	if (handle_OffsetZeroDone == NULL)
	{
		vPrintString("handle_OffsetZeroDone create failed.\n");
	}

	// Aanmaken van "SystemState queue"
	// Geeft de actuele status waarin de machine zich bevindt door.
	handle_StateQueue = xQueueCreate(QueueSize, sizeof(SystemState_t));
	if (handle_StateQueue == NULL)
	{
		vPrintString("handle_StateQueue create failed.\n");
	}
	
	// Aanmaken van "Offset queue"
	// Geeft de actuele X- en Y-offset van de frame-sensoren door.
	handle_OffsetQueue = xQueueCreate(QueueSize, sizeof(OffsetPos_t));
	if (handle_OffsetQueue == NULL)
	{
		vPrintString("handle_OffsetQueue create failed.\n");
	}

	// Aanmaken van "DisplayInfo queue"
	// Geeft twee algemene inforegels door aan het scherm.
	handle_DisplayInfoQueue = xQueueCreate(QueueSize, sizeof(DisplayInfo_t));
	if (handle_DisplayInfoQueue == NULL)
	{
		vPrintString("handle_DisplayInfoQueue create failed.\n");
	}

	/*
	// Aanmaken van "stroom queue"
	// Geeft de actuele stroomwaarde door.
	handle_stroomQueue = xQueueCreate(QueueSize, sizeof(float));
	if (handle_stroomQueue == NULL)
	{
		vPrintString("handle_stroomQueue create failed.\n");
	}
	*/
	
	/**************************************************** Taken aanmaken ****************************************************/
	// De taken worden aangemaakt en in de ready list geplaatst.
	// Prioriteit 0 is het laagste,
	// Prioriteit 4 is de hoogste mogelijk
	result = xTaskCreate(InputHandlerTask, "tsk_Input", (STACK_INPUT_TASK), NULL, 0, &handle_InputHandlerTask);
	if (result != pdPASS )
	{
		vPrintString("InputHandlerTask task create failed.\n");
	}
	result = xTaskCreate(ControlTask, "tsk_Control", (STACK_CONTROL_TASK), NULL, 2, &handle_ControlTask);
	if (result != pdPASS )
	{
		vPrintString("ControlTask task create failed.\n");
	}
	result = xTaskCreate(VisualisationTask, "tsk_Visualisation", (STACK_VISUALISATION_TASK), NULL, 1, &handle_VisualisationTask);
	if (result != pdPASS )
	{
		vPrintString("VisualisationTask task create failed.\n");

	}
	/**************************************************** Taken aanmaken ****************************************************/
}
