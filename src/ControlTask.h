/*
 * ControlTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef CONTROLTASK_H_
#define CONTROLTASK_H_


///////////////////////////////////////////////////////////////////////////////
#define CLOCK_PIN PIN_30
#define NOODSTOP_PIN PIN_50
#define NOODSCHAKEL_PIN PIN_51
#define ESCONFOUT_PIN PIN_52


#define LAMP_GREEN     0
#define LAMP_ORANGE    1
#define LAMP_RED       2

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