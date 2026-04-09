/*
 * ApplicationTasks.h
 *
 * Created: 27-11-2023 15:01:19
 *  Author: rasmsmee
 */ 


#ifndef APPLICATIONTASKS_H_
#define APPLICATIONTASKS_H_

///////////////////////////////////////////////////////////////////////////////
// objects made available for external use

extern EventGroupHandle_t	handle_ThreadEventGroup;
extern SemaphoreHandle_t	handle_RestartSemaphore;
extern QueueHandle_t		handle_ParameterQueue;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void StartApplicationTasks(void);


#endif /* APPLICATIONTASKS_H_ */