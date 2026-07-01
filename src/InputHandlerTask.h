/*
 * InputHandlerTask.h
 *
 * Created: 10/04/2023
 *  Author: Raph van Koeveringe
 */ 


#ifndef INPUTHANDLERTASK_H_
#define INPUTHANDLERTASK_H_

#include "bits.h"

// Event bits for handle_ButtonEventGroup
#define EVT_START_BUTTON         BIT_10
#define EVT_STOP_BUTTON          BIT_11
#define EVT_RESET_BUTTON         BIT_12
#define EVT_START_HOLD_BUTTON    BIT_13
#define EVT_STOP_HOLD_BUTTON     BIT_14
#define EVT_RESET_HOLD_BUTTON    BIT_15

///////////////////////////////////////////////////////////////////////////////
// function prototypes

///////////////////////////////////////////////////////////////////////////////
// local type definitions
typedef struct
{
	float x;
	float y;
} OffsetPos_t;

void InputHandlerTask(void *pvParameters);

#endif /* INPUTHANDLERTASK_H_ */
