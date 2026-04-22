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
static TaskHandle_t handle_VisualisationTask = NULL;
static TaskHandle_t handle_IMUTask = NULL;

///////////////////////////////////////////////////////////////////////////////
// global handles & objects

EventGroupHandle_t handle_ThreadEventGroup = NULL;
EventGroupHandle_t handle_ButtonEventGroup = NULL;

SemaphoreHandle_t  handle_EmergencySemaphore = NULL;

QueueHandle_t      handle_StateQueue = NULL;
TaskHandle_t       handle_ControlTask = NULL;



///////////////////////////////////////////////////////////////////////////////
// void StartApplicationTasks(void)
void StartApplicationTasks(void)
{
	BaseType_t result = pdFAIL;
	UBaseType_t StateQueueSize = 1;
	static const IMU_TASK_CONFIG imuTaskConfig = { I2C_CHANNEL_0, 0x68 };

	// Aanmaken van "SystemState queue"
	// Geeft de actuele status waarin de machine zich bevindt door.
	handle_StateQueue = xQueueCreate(StateQueueSize, sizeof(SystemState_t));
	if (handle_StateQueue == NULL)
	{
		vPrintString("handle_StateQueue create failed.\n");
	}
	
	// EmergencySmephore aanmaken
	handle_EmergencySemaphore = xSemaphoreCreateBinary();
	if (handle_EmergencySemaphore == NULL)
	{
		vPrintString("handle_EmergencySemaphore create failed.\n");
	}
	
	// Event group aanmaken voor knoppen, communicatie tussen ButtonHandlerTask -> ControlTask.
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
	

	

	
	/**************************************************** Taken aanmaken ****************************************************/
	// De taken worden aangemaakt en in de ready list geplaatst.
	// Prioriteit 0 is laagste 4 is hoogste!!
	result = xTaskCreate(ButtonHandlerTask, "tsk_Button", (configMINIMAL_STACK_SIZE), NULL, 0, &handle_ButtonHandlerTask);
	if (result == pdPASS )
	{
		vPrintString("ButtonHandlerTask task create failed.\n");
	}
	result = xTaskCreate(ControlTask, "tsk_Control", (configMINIMAL_STACK_SIZE), NULL, 2, &handle_ControlTask);
	if (result == pdPASS )
	{
		vPrintString("ControlTask task create failed.\n");
	}
	result = xTaskCreate(VisualisationTask, "tsk_Visualisation", (configMINIMAL_STACK_SIZE), NULL, 1, &handle_VisualisationTask);
	if (result == pdPASS )
	{
		vPrintString("VisualisationTask task create failed.\n");

	}

	// IMU initialisatie gebeurt in de task zelf, nadat de FreeRTOS scheduler draait.
	result = xTaskCreate(imu_Task,"task_UMI", (configMINIMAL_STACK_SIZE),(void *)&imuTaskConfig,1, &handle_IMUTask);
	if (result != pdPASS)
	{
		vPrintString("IMU task create failed.\n");
	}
	/**************************************************** Taken aanmaken ****************************************************/
}

