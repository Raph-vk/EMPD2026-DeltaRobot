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
#include "MotionEngine.h"
#include "ControlTask.h"
#include "MachinePins.h"


///////////////////////////////////////////////////////////////////////////////
// #defines
//Inputs
#define M1_LIMIT		BIT_M1_HOME
#define M2_LIMIT		BIT_M2_HOME
#define M3_LIMIT		BIT_M3_HOME
#define ESCON_OVERLOAD	BIT_NOOD

// 
static const uint8_t MotorHomeLimitBit[N_MOTORS] = {M1_LIMIT, M2_LIMIT, M3_LIMIT};
///////////////////////////////////////////////////////////////////////////////
// function prototypes

//void motor_EnableESCONController(void);
//void motor_DisableESCONController(void);

void motor_DisplayStatus(void);

Bool motor_IsHomeLimitActive(uint8_t motorIndex);
Bool homeAllMotors(void);     // homing-step voor alle 3 motoren

#endif /* MOTORCONTROL_H_ */
