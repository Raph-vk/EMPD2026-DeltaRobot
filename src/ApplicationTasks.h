/*
 * ApplicationTasks.h
 *
 * Created: 11-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef APPLICATIONTASKS_H_
#define APPLICATIONTASKS_H_

///////////////////////////////////////////////////////////////////////////////
// objects made available for external use
#include <asf.h>
// #include "IMUTask.h"

extern EventGroupHandle_t	handle_ThreadEventGroup;
extern EventGroupHandle_t	handle_ButtonEventGroup;
extern SemaphoreHandle_t	handle_NoodSemaphore;
extern QueueHandle_t		handle_StateQueue;
extern QueueHandle_t		handle_potQueue;
extern QueueHandle_t		handle_stroomQueue;
extern TaskHandle_t			handle_ControlTask;

//extern SemaphoreHandle_t	handle_clockInterruptSemaphore;

///////////////////////////////////////////////////////////////////////////////
// function prototypes
void StartApplicationTasks(void);


#endif /* APPLICATIONTASKS_H_ */
