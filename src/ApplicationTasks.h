/*
 * ApplicationTasks.h
 *
 * Created: 11-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef APPLICATIONTASKS_H_
#define APPLICATIONTASKS_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

///////////////////////////////////////////////////////////////////////////////
// shared queue payloads
#define DISPLAY_INFO_LINE_LENGTH	(32U)

typedef struct
{
	char regel1[DISPLAY_INFO_LINE_LENGTH];
	char regel2[DISPLAY_INFO_LINE_LENGTH];
} DisplayInfo_t;

///////////////////////////////////////////////////////////////////////////////
// objects made available for external use
extern EventGroupHandle_t	handle_ThreadEventGroup;
extern EventGroupHandle_t	handle_ButtonEventGroup;
extern SemaphoreHandle_t	handle_NoodSemaphore;
extern SemaphoreHandle_t	handle_OffsetZeroRequest;
extern SemaphoreHandle_t	handle_OffsetZeroDone;
extern QueueHandle_t		handle_StateQueue;
extern QueueHandle_t		handle_stroomQueue;
extern QueueHandle_t		handle_OffsetQueue;
extern QueueHandle_t		handle_DisplayInfoQueue;
//extern QueueHandle_t		handle_DisturbanceQueue;
extern TaskHandle_t			handle_ControlTask;

///////////////////////////////////////////////////////////////////////////////
// function prototypes
void StartApplicationTasks(void);
void DisplayInfo_Publish(const char *regel1, const char *regel2);
bool TcpCompensation_IsEnabled(void);
void TcpCompensation_SetEnabled(bool enabled);
bool TcpCompensation_Toggle(void);


#endif /* APPLICATIONTASKS_H_ */
