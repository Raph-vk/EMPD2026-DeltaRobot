/*
 * IOPorts.c
 *
 * Created: 1-10-2022 15:55:47
 *  Author: Roel Smeets
 */ 


///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"

///////////////////////////////////////////////////////////////////////////////
// file global objects

static uint32_t G_dioPins[NUM_PINS];

///////////////////////////////////////////////////////////////////////////////
//	void dio_InitPorts(void)

void dio_Init(void)
{
	int ix = 0;

	for (ix = MIN_PIN_NUMBER; ix <= MAX_PIN_NUMBER; ix++)
	{
		G_dioPins[ix] = 0;
	}

	// pins PWM2..PWM13 on Arduino Due
	// PWM4 and PWM10 are unused (CS for SD & Ethernet)
	// PWM13 is on-board yellow LED
	
	G_dioPins[PIN_PWM2]  = IOPORT_CREATE_PIN(PIOB, 25);
	G_dioPins[PIN_PWM3]  = IOPORT_CREATE_PIN(PIOC, 28);
	G_dioPins[PIN_PWM5]  = IOPORT_CREATE_PIN(PIOC, 25);
	G_dioPins[PIN_PWM6]  = IOPORT_CREATE_PIN(PIOC, 24);
	G_dioPins[PIN_PWM7]  = IOPORT_CREATE_PIN(PIOC, 23);
	G_dioPins[PIN_PWM8]  = IOPORT_CREATE_PIN(PIOC, 22);
	G_dioPins[PIN_PWM9]  = IOPORT_CREATE_PIN(PIOC, 21);
	G_dioPins[PIN_PWM11] = IOPORT_CREATE_PIN(PIOD,  7);
	G_dioPins[PIN_PWM12] = IOPORT_CREATE_PIN(PIOD,  8);
	
	G_dioPins[PIN_TX1]	 = IOPORT_CREATE_PIN(PIOA, 11);
	G_dioPins[PIN_RX1]	 = IOPORT_CREATE_PIN(PIOA, 10);

	G_dioPins[PIN_STATUS_LED] =  IOPORT_CREATE_PIN(PIOB,  27);

	// TWI (= I2C) pins on Arduino Due

	// TWI channel 1 on SAM chip = SDA / SCL ("channel 0") on Due
	G_dioPins[PIN_SDA0]	 = IOPORT_CREATE_PIN(PIOB, 12);
	G_dioPins[PIN_SCL0]	 = IOPORT_CREATE_PIN(PIOB, 13);

	// TWI channel 0 on SAM chip = SDA1 / SCL1 ("channel 1) on Due
	G_dioPins[PIN_SDA1]	 = IOPORT_CREATE_PIN(PIOA, 17);
	G_dioPins[PIN_SCL1]	 = IOPORT_CREATE_PIN(PIOA, 18);

	// pins A0..A7 on Arduino Due
	
	G_dioPins[PIN_A0] = IOPORT_CREATE_PIN(PIOA, 16);
	G_dioPins[PIN_A1] = IOPORT_CREATE_PIN(PIOA, 24);
	G_dioPins[PIN_A2] = IOPORT_CREATE_PIN(PIOA, 23);
	G_dioPins[PIN_A3] = IOPORT_CREATE_PIN(PIOA, 22);
	G_dioPins[PIN_A4] = IOPORT_CREATE_PIN(PIOA,  6);
	G_dioPins[PIN_A5] = IOPORT_CREATE_PIN(PIOA,  4);
	G_dioPins[PIN_A6] = IOPORT_CREATE_PIN(PIOA,  3);
	G_dioPins[PIN_A7] = IOPORT_CREATE_PIN(PIOA,  2);

	// pins A8..A11 on Arduino Due

	G_dioPins[PIN_A8]  = IOPORT_CREATE_PIN(PIOB,  17);
	G_dioPins[PIN_A9]  = IOPORT_CREATE_PIN(PIOB,  18);
	G_dioPins[PIN_A10] = IOPORT_CREATE_PIN(PIOB,  19);
	G_dioPins[PIN_A11] = IOPORT_CREATE_PIN(PIOB,  20);
	
	// digital IO pins 22..51 on Arduino Due
	// 52 and 53 are reserved (?)
	
	G_dioPins[PIN_22] = IOPORT_CREATE_PIN(PIOB, 26);
	G_dioPins[PIN_23] = IOPORT_CREATE_PIN(PIOA, 14);
	G_dioPins[PIN_24] = IOPORT_CREATE_PIN(PIOA, 15);
	G_dioPins[PIN_25] = IOPORT_CREATE_PIN(PIOD,  0);
	G_dioPins[PIN_26] = IOPORT_CREATE_PIN(PIOD,  1);
	G_dioPins[PIN_27] = IOPORT_CREATE_PIN(PIOD,  2);
	G_dioPins[PIN_28] = IOPORT_CREATE_PIN(PIOD,  3);
	G_dioPins[PIN_29] = IOPORT_CREATE_PIN(PIOD,  6);
	G_dioPins[PIN_30] = IOPORT_CREATE_PIN(PIOD,  9);
	G_dioPins[PIN_31] = IOPORT_CREATE_PIN(PIOA,  7);
	G_dioPins[PIN_32] = IOPORT_CREATE_PIN(PIOD, 10);
	G_dioPins[PIN_33] = IOPORT_CREATE_PIN(PIOC,  1);
	G_dioPins[PIN_34] = IOPORT_CREATE_PIN(PIOC,  2);
	G_dioPins[PIN_35] = IOPORT_CREATE_PIN(PIOC,  3);
	G_dioPins[PIN_36] = IOPORT_CREATE_PIN(PIOC,  4);
	G_dioPins[PIN_37] = IOPORT_CREATE_PIN(PIOC,  5);
	G_dioPins[PIN_38] = IOPORT_CREATE_PIN(PIOC,  6);
	G_dioPins[PIN_39] = IOPORT_CREATE_PIN(PIOC,  7);
	G_dioPins[PIN_40] = IOPORT_CREATE_PIN(PIOC,  8);
	G_dioPins[PIN_41] = IOPORT_CREATE_PIN(PIOC,  9);
	G_dioPins[PIN_42] = IOPORT_CREATE_PIN(PIOA, 19);
	G_dioPins[PIN_43] = IOPORT_CREATE_PIN(PIOA, 20);
	G_dioPins[PIN_44] = IOPORT_CREATE_PIN(PIOC, 19);
	G_dioPins[PIN_45] = IOPORT_CREATE_PIN(PIOC, 18);
	G_dioPins[PIN_46] = IOPORT_CREATE_PIN(PIOC, 17);
	G_dioPins[PIN_47] = IOPORT_CREATE_PIN(PIOC, 16);
	G_dioPins[PIN_48] = IOPORT_CREATE_PIN(PIOC, 15);
	G_dioPins[PIN_49] = IOPORT_CREATE_PIN(PIOC, 14);
	G_dioPins[PIN_50] = IOPORT_CREATE_PIN(PIOC, 13);
	G_dioPins[PIN_51] = IOPORT_CREATE_PIN(PIOC, 12);
	//TODO: 52 and 53
	
	ioport_pin_t ioPortPinNumber = 0;
	
	for (ix = MIN_PIN_NUMBER; ix <= MAX_PIN_NUMBER; ix++)
	{
		ioPortPinNumber = G_dioPins[ix];
		
		ioport_enable_pin(   ioPortPinNumber);
		ioport_set_pin_dir(  ioPortPinNumber, IOPORT_DIR_INPUT);	// always init as INPUT!!
		ioport_set_pin_level(ioPortPinNumber, false);				// and LOW level
	
		ioport_set_pin_mode( ioPortPinNumber, IOPORT_MODE_PULLUP);	// with pull-up
	}
}

///////////////////////////////////////////////////////////////////////////////
//	void dio_SetPinDirection(due_pin_number_t dioPinNumber, enum ioport_direction direction)

void dio_SetPinDirection(due_pin_number_t dioPinNumber, enum ioport_direction direction)
{
	if ( dio_IsValidPinNumber(dioPinNumber) )
	{
		ioport_set_pin_dir(G_dioPins[dioPinNumber], direction);
	}
}


///////////////////////////////////////////////////////////////////////////////
//	void dio_SetPinMode(due_pin_number_t dioPinNumber, ioport_mode_t pinMode)

void dio_SetPinMode(due_pin_number_t dioPinNumber, ioport_mode_t pinMode)
{
	if ( dio_IsValidPinNumber(dioPinNumber) )
	{
		ioport_set_pin_mode(G_dioPins[dioPinNumber], pinMode);
	}
}

///////////////////////////////////////////////////////////////////////////////
//	bool dio_IsValidPinNumber(due_pin_number_t dioPinNumber)

bool dio_IsValidPinNumber(due_pin_number_t dioPinNumber)
{
	bool result = false;
	
	result = (dioPinNumber>= MIN_PIN_NUMBER) && (dioPinNumber <= MAX_PIN_NUMBER);
	
	return result;
}

///////////////////////////////////////////////////////////////////////////////
//	void dio_TogglePin(due_pin_number_t dioPinNumber)

void dio_TogglePin(due_pin_number_t dioPinNumber)
{
	if ( dio_IsValidPinNumber(dioPinNumber) )
	{
		ioport_toggle_pin_level(G_dioPins[dioPinNumber]);
	}
}


///////////////////////////////////////////////////////////////////////////////
//	void dio_SetPin(due_pin_number_t dioPinNumber, bool highLevel)

void dio_SetPin(due_pin_number_t dioPinNumber, bool highLevel)
{
	if ( dio_IsValidPinNumber(dioPinNumber) )
	{
		ioport_set_pin_level(G_dioPins[dioPinNumber], highLevel);
	}
}

///////////////////////////////////////////////////////////////////////////////
//	bool dio_GetPin(due_pin_number_t dioPinNumber)

bool dio_GetPin(due_pin_number_t dioPinNumber)
{
	bool highLevel = false;
	
	if ( dio_IsValidPinNumber(dioPinNumber) )
	{
		highLevel = ioport_get_pin_level(G_dioPins[dioPinNumber]);
	}
	
	return highLevel;
}

///////////////////////////////////////////////////////////////////////////////
//	bool dio_GetIOPortPinNumber(due_pin_number_t dioPinNumber)

uint32_t dio_GetIOPortPinNumber(due_pin_number_t dioPinNumber)
{
	uint32_t ioPortPinNumber = 0;
	
	if ( dio_IsValidPinNumber(dioPinNumber) )
	{
		ioPortPinNumber = G_dioPins[dioPinNumber];
	}
	
	return ioPortPinNumber;
}
