/*
 * ApplicationTasks.h
 *
 * Created: 11-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef APPLICATIONTASKS_H_
#define APPLICATIONTASKS_H_

#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

///////////////////////////////////////////////////////////////////////////////
// objects made available for external use
extern EventGroupHandle_t	handle_ThreadEventGroup;
extern EventGroupHandle_t	handle_ButtonEventGroup;
extern SemaphoreHandle_t	handle_NoodSemaphore;
extern SemaphoreHandle_t	handle_OffsetZeroRequest;
extern SemaphoreHandle_t	handle_OffsetZeroDone;
extern QueueHandle_t		handle_StateQueue;
//extern QueueHandle_t		handle_potQueue;
//extern QueueHandle_t		handle_stroomQueue;
extern QueueHandle_t		handle_OffsetQueue;
//extern QueueHandle_t		handle_DisturbanceQueue;
extern TaskHandle_t			handle_ControlTask;

//extern SemaphoreHandle_t	handle_clockInterruptSemaphore;

///////////////////////////////////////////////////////////////////////////////
// function prototypes
void StartApplicationTasks(void);


#endif /* APPLICATIONTASKS_H_ */
