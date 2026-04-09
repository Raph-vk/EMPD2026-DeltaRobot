/*
 * MotroControl.c
 *
 * Created: 28-9-2023 15:37:48
 *  Author: rasmsmee
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DAC4921Lib.h"
#include "LEDLib.h"
#include "PortIOLib.h"

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "MotorControl.h"

///////////////////////////////////////////////////////////////////////////////
// void motor_DisplayStatus(void)

void motor_DisplayStatus(void)
{
	uint8_t portInValue = 0;
	uint8_t bitVal		= 0;
	bool isSet			= false;
	
	// non-inverting input port, pull-up resistors
	
	portInValue = port_GetInput();
	
	led_DisplayValue(portInValue >> 1);	// using bits 1..4

	vPrintString("digital input = 0x%02x\n", portInValue);
	
	isSet = port_IsBitSet(BIT_LIMIT_LEFT);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit Left:     %d\n", bitVal);

	isSet = port_IsBitSet(BIT_LIMIT_RIGHT);
	bitVal = isSet? 1 : 0;
	vPrintString("Limit Right:    %d\n", bitVal);

	isSet = port_IsBitSet(BIT_ATOM_ERROR);
	bitVal = isSet? 1 : 0;
	vPrintString("Atom Error:     %d\n", bitVal);

	isSet = port_IsBitSet(BIT_ESCON_OVERLOAD);
	bitVal = isSet? 1 : 0;
	vPrintString("ESCON Overload: %d\n", bitVal);
	
	vPrintString("\n");
}


///////////////////////////////////////////////////////////////////////////////
// bool motor_HasOverload(void)

bool motor_HasOverload(void)
{
	bool overload = true;

	overload = port_IsBitSet(BIT_ESCON_OVERLOAD);
	
	return overload;
}


///////////////////////////////////////////////////////////////////////////////
// bool motor_IsAtLimit(motor_direction_t direction)
// return atLimit (TRUE/FALSE) voor bewegingsrichting
bool motor_IsAtLimit(motor_direction_t direction)
{
	bool atLimit = true;	// safe: assume at limit true
	
	//Als richting is, check BIT. BIT nummer/mask is ingesteld in headerfile.
	if (direction == MOVE_LEFT)
	{
		atLimit = port_IsBitSet(BIT_LIMIT_LEFT);
	}
	else if (direction == MOVE_RIGHT)
	{
		atLimit = port_IsBitSet(BIT_LIMIT_RIGHT);
	}
	
	if (atLimit)
	{
		led_DisplayValue(0x0F); // Alle leds aan.
	}
	
	return atLimit;
}


///////////////////////////////////////////////////////////////////////////////
// bool motor_Move(motor_direction_t direction)
//
// returns false if motor already at limit: movement NOT allowed
// returns true if motor not at limit: movement IS allowed

bool motor_Move(motor_direction_t direction)
{
	bool alreadyAtLimit = true;
	uint8_t dacChannel  = 0;
	float dacOutputVoltageLeft  =  -4.0;
	float dacOutputVoltageRight =   6.0;
	
	// Check of de motor niet al op limiet is.
	alreadyAtLimit = motor_IsAtLimit(direction);
	
	// only move motor if NOT at limit:
	if (alreadyAtLimit == false)
	{
		
		// Juiste richting op verplaatsen 
		if (direction == MOVE_LEFT)
		{
			led_DisplayValue(0x08);	// left LED on (bit: 1000)
			dac_SetOutputVoltage(dacChannel, dacOutputVoltageLeft); 
		}
		else if (direction == MOVE_RIGHT)
		{
			led_DisplayValue(0x01);	// right LED on (bit: 0001)
			dac_SetOutputVoltage(dacChannel, dacOutputVoltageRight);
		}
	}
	else	// safe default action if already at limit: stop
	{
		led_DisplayValue(0x00);
		motor_Stop();
	}
	
	return alreadyAtLimit;
}

///////////////////////////////////////////////////////////////////////////////
// void MotorGotoHomePosition(motor_direction_t direction)
//
// go to home position, either left or right

void motor_GotoHomePosition(motor_direction_t direction)
{
	// Beweeg motor tot limiet in die richting.
	motor_Move(direction);
	while (motor_IsAtLimit(direction) == false)
	{
		// do nothing, just keep going...
	}
	motor_Stop();
	taskSleep(1000);	// allow for mechanical debounce...
}


///////////////////////////////////////////////////////////////////////////////
// void motor_Stop(void)
//
// stop motor, set DAC output channel 0 to 0 Volt

void motor_Stop(void)
{
	uint8_t dacChannel  = 0;
	float	dacValue	= 0.0;
	
	dac_SetOutputVoltage(dacChannel, dacValue);
	
	led_DisplayValue(0x00); // alle leds uit.
}


///////////////////////////////////////////////////////////////////////////////
// void motor_EnableESCONController(void)
//
//enable ESCON controller via output port bit 0

void motor_EnableESCONController(void)
{
	port_SetBit(BIT_ESCON_ENABLE, true);
}

///////////////////////////////////////////////////////////////////////////////
// void motor_DisableESCONController(void)
//
// disable ESCON controller via output port bit 0

void motor_DisableESCONController(void)
{
	port_SetBit(BIT_ESCON_ENABLE, false);
}
