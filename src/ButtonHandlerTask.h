/*
 * ButtonHandlerTask.h
 *
 * Created: 23-11-2023 13:17:49
 *  Author: rasmsmee
 */ 


#ifndef BUTTONHANDLERTASK_H_
#define BUTTONHANDLERTASK_H_

///////////////////////////////////////////////////////////////////////////////
// Bits for handle_ButtonEventGroup;
#define BIT_START_BUTTON    BIT_10
#define BIT_STOP_BUTTON     BIT_11
#define BIT_RESET_BUTTON    BIT_12

///////////////////////////////////////////////////////////////////////////////
// objects made available for external use
// Semaphore wordt ergens anders aangemaakt maar in ButtonHandlerTask.c ook gebruikt.
extern SemaphoreHandle_t handle_EmergencySemaphore;
extern EventGroupHandle_t	handle_ThreadEventGroup;
extern EventGroupHandle_t	handle_ButtonEventGroup;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void ButtonHandlerTask(void *pvParameters);
void EmergencyInterruptHandler(void *pvParameters);

#endif /* BUTTONHANDLERTASK_H_ */