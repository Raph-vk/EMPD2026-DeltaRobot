/*
 * IOPortLib.h
 *
 * Created: 1-10-2022 16:02:48
 *  Author: Roel Smeets
 *
 * RASM, changed 16-04-2026:
 * added PIN_TX3 and PIN_RX3 for use with external SPI bus
 */ 
#ifndef DEVICEIOLIB_H_
#define DEVICEIOLIB_H_

//RAPH ADDED
#include <stdbool.h>
#include <stdint.h>
#include <ioport.h>

///////////////////////////////////////////////////////////////////////////////
// defines

// PIN_PWM4  reserved: Slave Select 0 on SPI = SD card
// PIN_PWM10 reserved: Slave Select 1 on SPI = Ethernet
// PWM13 is on-board yellow LED

typedef enum pin_number_t
{
	MIN_PIN_NUMBER = 0,
	
	PIN_A0		   = 0,
	PIN_A1,
	PIN_A2,
	PIN_A3,
	PIN_A4,
	PIN_A5,
	PIN_A6,
	PIN_A7,
	PIN_A8,
	PIN_A9,
	PIN_A10,
	PIN_A11,
	
	PIN_DAC0,
	PIN_DAC1,
	
	PIN_CANRX0,
	PIN_CANTX0,
	
	PIN_PWM2,
	PIN_PWM3,
	// PIN_PWM4  reserved: Slave Select 0 on SPI = SD card
	PIN_PWM5,
	PIN_PWM6,
	PIN_PWM7,
	PIN_PWM8,
	PIN_PWM9,
	// PIN_PWM10 reserved: Slave Select 1 on SPI = Ethernet
	PIN_PWM11,
	PIN_PWM12,
	
	// PWM13 is on-board yellow LED
	PIN_STATUS_LED,
	
	PIN_TX1,
	PIN_RX1,

	PIN_TX3,	// SPISEL6*
	PIN_RX3,	// SPISEL7*
	
	PIN_SDA0,	// I2C port 0
	PIN_SCL0,
	PIN_SDA1,	// I2C port 1
	PIN_SCL1,
	
	PIN_22,
	PIN_23,
	PIN_24,
	PIN_25,
	PIN_26,
	PIN_27,
	PIN_28,
	PIN_29,
	PIN_30,
	PIN_31,
	PIN_32,
	PIN_33,
	PIN_34,
	PIN_35,
	PIN_36,
	PIN_37,
	PIN_38,
	PIN_39,

	PIN_40,
	PIN_41,
	PIN_42,
	PIN_43,
	PIN_44,
	PIN_45,
	PIN_46,
	PIN_47,
	PIN_48,
	PIN_49,

	PIN_50,
	PIN_51,
	PIN_52,
	PIN_53,
	
	MAX_PIN_NUMBER = PIN_53,
	NUM_PINS,	// one more than last pin number!
} due_pin_number_t;


///////////////////////////////////////////////////////////////////////////////
//	function prototypes

void dio_Init(void);
bool dio_IsValidPinNumber(due_pin_number_t dioPinNumber);
void dio_TogglePin(due_pin_number_t dioPinNumber);
void dio_SetPin(due_pin_number_t dioPinNumber, bool highLevel);
bool dio_GetPin(due_pin_number_t dioPinNumber);
void dio_SetPinDirection(due_pin_number_t dioPinNumber, enum ioport_direction direction);
void dio_SetPinMode(due_pin_number_t dioPinNumber, ioport_mode_t pinMode);

uint32_t dio_GetIOPortPinNumber(due_pin_number_t dioPinNumber);

#endif /* IOPORTLIB_H_ */