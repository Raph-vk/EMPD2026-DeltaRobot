/*
 * MotorControl.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// function prototypes

//void motor_EnableESCONController(void);
//void motor_DisableESCONController(void);

bool motor_IsHomeLimitActive(uint8_t motorIndex);
bool anyHomeSwitchActive(void);
bool homeAllMotors(void);     // homing-step voor alle 3 motoren

#endif /* MOTORCONTROL_H_ */
