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
#include "MachinePins.h"

///////////////////////////////////////////////////////////////////////////////
// #defines

//Outputs
#define ESCON_ENABLE	BIT_ESCON_ENABLE	// bit positions in control output port
// DAC0, DAC1, DAC2; Motor analoge voltages naar EsconModule.

//Inputs
#define M1_LIMIT		BIT_M1_HOME
#define M2_LIMIT		BIT_M2_HOME
#define M3_LIMIT		BIT_M3_HOME
#define ESCON_OVERLOAD	BIT_ESON_OVERLOAD

///////////////////////////////////////////////////////////////////////////////
// function prototypes


void motor_EnableESCONController(void);
void motor_DisableESCONController(void);
bool motor_HasOverload(void);

void motor_DisplayStatus(void);
//void MotorHold(uint8_t motorIndex);

bool homeAllMotors(void);     // homing-step voor alle 3 motoren

#endif /* MOTORCONTROL_H_ */
