/*
 * ControlTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef CONTROLTASK_H_
#define CONTROLTASK_H_


///////////////////////////////////////////////////////////////////////////////
#include "MachinePins.h"

#define LAMP_GREEN     BIT_LAMP_GREEN
#define LAMP_ORANGE    BIT_LAMP_ORANGE
#define LAMP_RED       BIT_LAMP_RED

///////////////////////////////////////////////////////////////////////////////
// local type definitions
typedef enum
{
	STATE_INIT = 0,
	STATE_WAIT,
	STATE_HOMING,
	STATE_READY,
	STATE_RUNNING,
	STATE_FAULT
} SystemState_t;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void ClockInterruptHandler(uint32_t id, uint32_t mask);
void EmergencyInterruptHandler(uint32_t id, uint32_t mask);
void ControlTask(void *pvParameters);

void port_AllLampsOff(void);

#endif /* CONTROLTASK_H_ */