/*
 * ControlTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#ifndef CONTROLTASK_H_
#define CONTROLTASK_H_

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
bool InNoodsituatie(void);

void ClockInterruptHandler(uint32_t id, uint32_t mask);
void NoodInterruptHandler(uint32_t id, uint32_t mask);

void ControlTask(void *pvParameters);




#endif /* CONTROLTASK_H_ */
