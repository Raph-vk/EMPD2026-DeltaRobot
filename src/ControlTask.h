/*
 * ControlTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#ifndef CONTROLTASK_H_
#define CONTROLTASK_H_

#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// local type definitions STATEMACHINE
typedef enum
{
	STATE_INIT = 0,
	STATE_WAIT,
	STATE_HOMING,
	STATE_READY,
	STATE_TEACH,
	STATE_RUNNING,
	STATE_PAUSE,
	STATE_FAULT
} SystemState_t;

///////////////////////////////////////////////////////////////////////////////
// function prototypes
bool InNoodsituatie(void);
void ToState(SystemState_t newState);
void ControlTask(void *pvParameters);




#endif /* CONTROLTASK_H_ */
