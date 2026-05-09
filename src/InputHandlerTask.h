/*
 * InputHandlerTask.h
 *
 * Created: 10/04/2023
 *  Author: Raph van Koeveringe
 */ 


#ifndef INPUTHANDLERTASK_H_
#define INPUTHANDLERTASK_H_

///////////////////////////////////////////////////////////////////////////////
// library includes
#include "bits.h"
#include "ADCLib.h"


///////////////////////////////////////////////////////////////////////////////
// Event bits for handle_ThreadEventGroup
// BIT_0 (BUTTON) && BIT_1 (VisualisationTask)

// Event bits for handle_ButtonEventGroup
#define EVT_START_BUTTON    BIT_10
#define EVT_STOP_BUTTON     BIT_11
#define EVT_RESET_BUTTON    BIT_12




///////////////////////////////////////////////////////////////////////////////
// function prototypes

void InputHandlerTask(void *pvParameters);

#endif /* INPUTHANDLERTASK_H_ */
