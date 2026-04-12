/*
 * ButtonHandlerTask.h
 *
 * Created: 23-11-2023 13:17:49
 *  Author: rasmsmee
 */ 


#ifndef BUTTONHANDLERTASK_H_
#define BUTTONHANDLERTASK_H_

#include "bits.h"

///////////////////////////////////////////////////////////////////////////////
// Event bits for handle_ButtonEventGroup
#define EVT_START_BUTTON    BIT_10
#define EVT_STOP_BUTTON     BIT_11
#define EVT_RESET_BUTTON    BIT_12




///////////////////////////////////////////////////////////////////////////////
// function prototypes

void ButtonHandlerTask(void *pvParameters);

#endif /* BUTTONHANDLERTASK_H_ */
