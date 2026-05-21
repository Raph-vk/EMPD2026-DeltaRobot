/*
 * MotorControl.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "LEDLib.h"
#include "PortIOLib.h"
#include "DAC4921Lib.h"
#include "QC7366Lib.h" // LS7366R encoder counter

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "QuadratureCounters.h"
#include "ControlTask.h"
#include "MachinePins.h"
#include "Regelaar.h"


///////////////////////////////////////////////////////////////////////////////
static const uint8_t MotorHomeLimitBit[N_MOTORS] = {BIT_M1_HOME, BIT_M2_HOME, BIT_M3_HOME};
///////////////////////////////////////////////////////////////////////////////
// function prototypes

//void motor_EnableESCONController(void);
//void motor_DisableESCONController(void);

void motor_DisplayStatus(void);
Bool motor_IsHomeLimitActive(uint8_t motorIndex);
Bool anyHomeSwitchActive(void);
Bool homeAllMotors(void);     // homing-step voor alle 3 motoren

#endif /* MOTORCONTROL_H_ */
