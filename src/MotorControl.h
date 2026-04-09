/*
 * MotorControl.h
 *
 * Created: 28-9-2023 15:39:41
 *  Author: rasmsmee
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

///////////////////////////////////////////////////////////////////////////////
// #defines

#define BIT_LIMIT_LEFT		1	// bit positions in status input port
#define BIT_LIMIT_RIGHT		2
#define BIT_ATOM_ERROR		3
#define BIT_ESCON_OVERLOAD	4

#define BIT_ESCON_ENABLE	0	// bit positions in control output port
#define BIT_ESCON_POWERON	1

///////////////////////////////////////////////////////////////////////////////
// typedefs

typedef enum
{
	MOVE_LEFT,
	MOVE_RIGHT,
} motor_direction_t;


///////////////////////////////////////////////////////////////////////////////
// function prototypes

void motor_DisplayStatus(void);
bool motor_HasOverload(void);
bool motor_IsAtLimit(motor_direction_t direction);
bool motor_Move(motor_direction_t direction);
void motor_Stop(void);
void motor_GotoHomePosition(motor_direction_t direction);
void motor_EnableESCONController(void);
void motor_DisableESCONController(void);

#endif /* MOTORCONTROL_H_ */
