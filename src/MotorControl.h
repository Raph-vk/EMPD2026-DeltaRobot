/*
 * MotorControl.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

///////////////////////////////////////////////////////////////////////////////
// #includes
#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// #defines

#define N_MOTORS    3

#define BIT_M1_LIMIT		1	// bit positions in status input port
#define BIT_M2_LIMIT		2
#define BIT_M3_LIMIT		3
//#define BIT_ATOM_ERROR	3
#define BIT_ESCON_OVERLOAD	4

#define BIT_ESCON_ENABLE	0	// bit positions in control output port
#define BIT_ESCON_POWERON	1

///////////////////////////////////////////////////////////////////////////////
// typedefs

//typedef enum
//{
//	MOVE_UP,
//	MOVE_DOWN,
//} motor_direction_t;


///////////////////////////////////////////////////////////////////////////////
// function prototypes


void QCEncodersSetup(void)
void QCEncodersClearCount(uint8_t qcChannel)


void motor_EnableESCONController(void);
void motor_DisableESCONController(void);
bool motor_HasOverload(void);

void motor_DisplayStatus(void);

bool motor_IsHomeLimitActive(uint8_t motorIndex);
//void MotorHold(uint8_t motorIndex);

bool motors_Move(void);              // homing-step voor alle 3 motoren

#endif /* MOTORCONTROL_H_ */