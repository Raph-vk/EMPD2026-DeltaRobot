/*
 * InputHandlerTask.h
 *
 * Created: 23-11-2023 13:17:49
 *  Author: rasmsmee
 */ 


#ifndef INPUTHANDLERTASK_H_
#define INPUTHANDLERTASK_H_

///////////////////////////////////////////////////////////////////////////////
// objects made available for external use
// Semaphore wordt ergens anders aangemaakt maar in ButtonHandlerTask.c ook gebruikt.
extern SemaphoreHandle_t handle_StartSemaphore;
extern SemaphoreHandle_t handle_StopSemaphore;
extern SemaphoreHandle_t handle_ResetSemaphore;
extern SemaphoreHandle_t handle_NoodSemaphore;


///////////////////////////////////////////////////////////////////////////////
// function prototypes

void InputHandlerTask(void *pvParameters);


#endif /* BUTTONHANDLERTASK_H_ */