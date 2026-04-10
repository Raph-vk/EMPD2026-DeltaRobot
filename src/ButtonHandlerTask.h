/*
 * ButtonHandlerTask.h
 *
 * Created: 23-11-2023 13:17:49
 *  Author: rasmsmee
 */ 


#ifndef BUTTONHANDLERTASK_H_
#define BUTTONHANDLERTASK_H_

///////////////////////////////////////////////////////////////////////////////
// objects made available for external use
// Semaphore wordt ergens anders aangemaakt maar in ButtonHandlerTask.c ook gebruikt.
extern SemaphoreHandle_t handle_EmergenySemaphore;
extern EventGroupHandle_t	handle_ThreadEventGroup;
extern EventGroupHandle_t	handle_ButtonEventGroup;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void ButtonHandlerTask(void *pvParameters);
void EmergencyInterruptHandler(void *pvParameters);

#endif /* BUTTONHANDLERTASK_H_ */