/*
 * PortIOLib.c
 *
 * Created: 13-10-2022 21:51:28
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"
#include "PortIOLib.h"


///////////////////////////////////////////////////////////////////////////////
// table with Arduino Due pin assignment for input pins

static const uint8_t G_InputPins[N_INPUT_PINS] =
{
	PIN_30,	// LSB, bit 0
	PIN_31,
	PIN_32,
	PIN_33,
	PIN_34,
	PIN_35,
	PIN_36,
	PIN_37,	// MSB, bit 7
};


///////////////////////////////////////////////////////////////////////////////
// table with Arduino Due pin assignment for output pins

static const uint8_t G_OutputPins[N_OUTPUT_PINS] =
{
	PIN_38,	// LSB, bit 0
	PIN_39,
	PIN_40,
	PIN_41,
	PIN_42,
	PIN_43,
	PIN_44,
	PIN_45,	// MSB, bit 7
};


///////////////////////////////////////////////////////////////////////////////
// void port_Init(void)

void port_Init(void)
{
	uint8_t pin = 0;
	
	// initialize 8-bit input port
	
	for (pin = 0; pin < N_INPUT_PINS; pin++)
	{
		dio_SetPinDirection(G_InputPins[pin], IOPORT_DIR_INPUT);
		ioport_set_pin_mode(G_InputPins[pin], IOPORT_MODE_PULLUP);	// with pull-up
	}	
	
	// initialize 8-bit output port

	for (pin = 0; pin < N_OUTPUT_PINS; pin++)
	{
		dio_SetPinDirection(G_OutputPins[pin], IOPORT_DIR_OUTPUT);
		dio_SetPin(G_OutputPins[pin], false);	// output is LOW
	}
	
}

///////////////////////////////////////////////////////////////////////////////
// uint8_t port_GetInput(void)

uint8_t port_GetInput(void)
{
	uint8_t pin = 0;
	uint8_t value = 0;
	
	for (pin = 0; pin < N_INPUT_PINS; pin++)
	{
		if (dio_GetPin(G_InputPins[pin]) == true)
		{
			value = value | (0x01 << pin);
		}
	}
	
	return value;
}


///////////////////////////////////////////////////////////////////////////////
// bool port_IsValidBitNumber(uint8_t bitNumber)

bool port_IsValidBitNumber(uint8_t bitNumber)
{
	return (bitNumber < 8);
}


///////////////////////////////////////////////////////////////////////////////
// bool port_IsBitSet(uint8_t bitNumber)

bool port_IsBitSet(uint8_t bitNumber)
{
	bool isSet = false;
	
	if (port_IsValidBitNumber(bitNumber))
	{
		isSet = dio_GetPin(G_InputPins[bitNumber]);
	}
	
	return isSet;
}

///////////////////////////////////////////////////////////////////////////////
// void port_WriteOutput(uint8_t value)

void port_WriteOutput(uint8_t value)
{
	uint8_t bitNr = 0;
	bool bitOn	  = false;
	
	for (bitNr = 0; bitNr < N_OUTPUT_PINS; bitNr++)
	{
		if ((value & (0x01 << bitNr)) != 0)
		{
			bitOn = true;
		}
		else
		{
			bitOn = false;
		}
		dio_SetPin(G_OutputPins[bitNr], bitOn);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void port_SetBit(uint8_t bitNumber, bool bitOn)

void port_SetBit(uint8_t bitNumber, bool bitOn)
{
	if (port_IsValidBitNumber(bitNumber))
	{
		dio_SetPin(G_OutputPins[bitNumber], bitOn);
	}
}
