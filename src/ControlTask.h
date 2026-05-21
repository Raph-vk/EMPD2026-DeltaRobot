/*
 * ControlTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#ifndef CONTROLTASK_H_
#define CONTROLTASK_H_
///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <string.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DeviceIOLib.h"
#include "InterruptLib.h"
#include "bits.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "MotorControl.h"
#include "InputHandlerTask.h"
#include "MotionEngine.h"
#include "ApplicationTasks.h"
#include "MachinePins.h"
#include "MotionEngine.h"

///////////////////////////////////////////////////////////////////////////////
// local type definitions STATEMACHINE
typedef enum
{
	STATE_INIT = 0,
	STATE_WAIT,
	STATE_HOMING,
	STATE_READY,
	STATE_PAUSE,
	STATE_RUNNING,
	STATE_FAULT
} SystemState_t;

///////////////////////////////////////////////////////////////////////////////
// function prototypes
void ToState(SystemState_t newState);
Bool InNoodsituatie(void);

void ClockInterruptHandler(uint32_t id, uint32_t mask);
void NoodInterruptHandler(uint32_t id, uint32_t mask);

void ControlTask(void *pvParameters);




#endif /* CONTROLTASK_H_ */
