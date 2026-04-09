/*
 * SwitchLib.c
 *
 * Created: 13-10-2022 19:11:47
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"
#include "SwitchLib.h"

///////////////////////////////////////////////////////////////////////////////
// table with Arduino Due pin assignment for switches (active LOW)

static const uint8_t G_SwitchPins[N_SWITCHES] =
{
	PIN_A9,	// switch 1 = LSB, bit 0
	PIN_A8,	// switch 2
	PIN_A7,	// switch 3
	PIN_A6,	// switch 4 = MSB, bit 3
};


///////////////////////////////////////////////////////////////////////////////
// void switch_Init(void)

void switch_Init(void)
{
	uint8_t sw = 0;
	
	for (sw = 0; sw < N_SWITCHES; sw++)
	{
		dio_SetPinDirection(G_SwitchPins[sw], IOPORT_DIR_INPUT);	// input
		dio_SetPinMode(G_SwitchPins[sw],	  IOPORT_MODE_PULLUP);	// with pull-up
	}	
}

///////////////////////////////////////////////////////////////////////////////
// bool switch_IsValidNumber(uint8_t switchNumber)

bool switch_IsValidNumber(uint8_t switchNumber)
{
	return (switchNumber < N_SWITCHES);
}


///////////////////////////////////////////////////////////////////////////////
// bool switch_IsPressed(uint8_t switchNumber)

bool switch_IsPressed(uint8_t switchNumber)
{
	bool isPressed = false;
	
	if (switch_IsValidNumber(switchNumber))
	{
		//active LOW: pressed == 0 !!
		isPressed = (dio_GetPin(G_SwitchPins[switchNumber]) == false);
	}
	
	return isPressed;
}

///////////////////////////////////////////////////////////////////////////////
// uint8_t switch_GetValue(void)

uint8_t switch_GetValue(void)
{
	uint8_t value = 0;
	uint8_t sw = 0;
	
	for (sw = 0; sw < N_SWITCHES; sw++)
	{
		if (switch_IsPressed(sw))
		{
			value = value | (0x01 << sw);
		}
	}
	
	return value;
}
