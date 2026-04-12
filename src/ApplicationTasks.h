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

extern EventGroupHandle_t	handle_ThreadEventGroup;
extern EventGroupHandle_t	handle_ButtonEventGroup;
extern SemaphoreHandle_t	handle_EmergencySemaphore;
extern QueueHandle_t		handle_StateQueue;
extern TaskHandle_t			handle_ControlTask;


///////////////////////////////////////////////////////////////////////////////
// function prototypes

void StartApplicationTasks(void);


#endif /* APPLICATIONTASKS_H_ */
