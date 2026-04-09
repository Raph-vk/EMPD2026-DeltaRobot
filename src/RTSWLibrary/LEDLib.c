/*
 * LEDLib.c
 *
 * Created: 13-10-2022 14:27:51
 *  Author: rasmsmee
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"
#include "LEDLib.h"


///////////////////////////////////////////////////////////////////////////////
// table with Arduino Due pin assignment for LED's

static const uint8_t G_LEDPins[N_LEDS] =
{
	PIN_PWM8,	// LED D1 = LSB
	PIN_PWM9,	// LED D2
	PIN_PWM11,	// LED D3
	PIN_PWM12,	// LED D4 = MSB
};

///////////////////////////////////////////////////////////////////////////////
// void led_Init(void)

void led_Init(void)
{
	uint8_t led = 0;
	
	for (led = 0; led < N_LEDS; led++)
	{
		dio_SetPinDirection(G_LEDPins[led], IOPORT_DIR_OUTPUT);
		dio_SetPin(G_LEDPins[led], false);	// output is LOW
	}
}


///////////////////////////////////////////////////////////////////////////////
// void led_DisplayValue(uint8_t value)

void led_DisplayValue(uint8_t value)
{
	uint8_t ix	   = 0;
	uint8_t ledPin = 0;
	bool	ledOn  = false;
	
	for (ix = 0; ix < N_LEDS; ix++)
	{
		if ( (value & (0x01 << ix)) != 0) 
		{
			ledOn = true;
		}
		else
		{
			ledOn = false;
		}
		ledPin = G_LEDPins[ix];
		dio_SetPin(ledPin, ledOn);
	}
}

///////////////////////////////////////////////////////////////////////////////
// bool led_IsValidNumber(uint8_t ledNumber)

bool led_IsValidNumber(uint8_t ledNumber)
{
	return (ledNumber < N_LEDS);	
}

///////////////////////////////////////////////////////////////////////////////
// void led_SetState(uint8_t ledNumber, bool ledOn)

void led_SetState(uint8_t ledNumber, bool ledOn)
{
	uint8_t ledPin = 0;
	
	if (led_IsValidNumber(ledNumber))
	{
		ledPin = G_LEDPins[ledNumber];
		dio_SetPin(ledPin, ledOn);		
	}
	
}