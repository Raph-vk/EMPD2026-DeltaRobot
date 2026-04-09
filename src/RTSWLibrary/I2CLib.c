/*
 * I2CLib.c
 *
 * Created: 17-10-2022 19:05:36
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"
#include "I2CLib.h"
#include "vPrintString.h"

///////////////////////////////////////////////////////////////////////////////
// file global data

static I2C_HANDLE G_i2cChannelHandles[I2C_N_CHANNELS];

static bool G_IsI2CInitialised = false;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

///////////////////////////////////////////////////////////////////////////////
// bool i2c_IsValidChannelNumber(I2C_CHANNEL i2cChannelNumber)

bool i2c_IsValidChannelNumber(I2C_CHANNEL i2cChannelNumber)
{
	return (i2cChannelNumber < I2C_N_CHANNELS);
}

///////////////////////////////////////////////////////////////////////////////
// I2C_HANDLE *i2c_GetChannelHandle(I2C_CHANNEL i2cChannelNumber)

I2C_HANDLE *i2c_GetChannelHandle(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *handle = NULL;
	
	if (i2c_IsValidChannelNumber(i2cChannelNumber))
	{
		handle = &G_i2cChannelHandles[i2cChannelNumber];
	}	
	else
	{
		handle = NULL;
	}
	
	return handle;
}

///////////////////////////////////////////////////////////////////////////////
// void i2c_Init(void)
//
// initializes both I2C channels on the Arduino Due
// returns true if both channels successfully assigned, false if one or both 
// fail

bool i2c_Init(void)
{
	uint8_t channel	= 0;
	bool i2cChannelInitOK = true;
	bool initOk	= true;		//assume success

	if (G_IsI2CInitialised == false)	// prevents multiple inits
	{
		// always do a hardware bit-bang reset of the I2C bus on 
		// init in case of lock-ups...
		
		i2c_ClearBus(I2C_CHANNEL_0);
		i2c_ClearBus(I2C_CHANNEL_1);

		memset(G_i2cChannelHandles, 0, sizeof(G_i2cChannelHandles));

		// yes, physical channel 0 on the SAM3X8E is labeled channel 1 on the Arduino Due...

		channel = 0;
		G_i2cChannelHandles[channel].ioDesc.scl = dio_GetIOPortPinNumber(PIN_SCL1);
		G_i2cChannelHandles[channel].ioDesc.sda = dio_GetIOPortPinNumber(PIN_SDA1);
		G_i2cChannelHandles[channel].twi		= NULL;

		i2cChannelInitOK = i2c_InitChannel(I2C_CHANNEL_0);
		if (i2cChannelInitOK == false)
		{
			initOk = false;
		}
	
		// and physical channel 1 on the SAM3X8E is labeled channel 0 on the Arduino Due...

		channel = 1;
		G_i2cChannelHandles[channel].ioDesc.scl = dio_GetIOPortPinNumber(PIN_SCL0);
		G_i2cChannelHandles[channel].ioDesc.sda = dio_GetIOPortPinNumber(PIN_SDA0);
		G_i2cChannelHandles[channel].twi		 = NULL;

		i2cChannelInitOK = i2c_InitChannel(I2C_CHANNEL_1);
		if (i2cChannelInitOK == false)
		{
			initOk = false;
		}
		
		G_IsI2CInitialised = initOk;
	}
	
	return initOk;
}


///////////////////////////////////////////////////////////////////////////////
// bool i2c_InitChannel(I2C_CHANNEL channel)
 
bool i2c_InitChannel(I2C_CHANNEL channel)
{
	I2C_HANDLE *handle		   = NULL;
	uint8_t	   *receive_buffer = NULL;
	bool initOk = true;
	
	if( i2c_IsValidChannelNumber(channel) == false )
	{
		return false;
	}

	handle = &G_i2cChannelHandles[channel];
	
	if (handle->twi == NULL)	// I2C channel not assigned yet!
	{
		ioport_disable_pin(handle->ioDesc.sda);
		ioport_disable_pin(handle->ioDesc.scl);
		
		receive_buffer = pvPortMalloc(I2C_RECEIVE_BUFFER_SIZE);
		if (receive_buffer != NULL)
		{
			handle->i2c_options.interrupt_priority  = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY; //of deze configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1,
			handle->i2c_options.options_flags	    = (USE_TX_ACCESS_SEM | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE);
			handle->i2c_options.receive_buffer      = receive_buffer;
			handle->i2c_options.receive_buffer_size = I2C_RECEIVE_BUFFER_SIZE;
			handle->i2c_options.operation_mode		= TWI_I2C_MASTER;
		
			handle->twi = freertos_twi_master_init((channel == I2C_CHANNEL_0) ? TWI0 : TWI1, &handle->i2c_options);
			twi_set_speed(handle->twi, I2C_FAST_MODE_SPEED, sysclk_get_peripheral_hz());
			
			initOk = true;
		}
		else
		{
			initOk = false;
		}
	}

	return initOk;
}

///////////////////////////////////////////////////////////////////////////////
// void i2c_WriteRegisters(I2C_HANDLE *handle,
//		uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t *buffer, uint8_t len)

void i2c_WriteRegisters(I2C_HANDLE *i2cHandle, 
		uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t *buffer, uint8_t len)
{
	portTickType maxBlockTimeTicks = (200UL / portTICK_RATE_MS);
	
	twi_packet_t p_packet;

	p_packet.chip			= i2cBaseAddress;
	p_packet.addr[0]		= i2cSubaddress;
	p_packet.addr_length	= 1;
	p_packet.buffer			= buffer;
	p_packet.length			= len;

	freertos_twi_write_packet_async(i2cHandle->twi, &p_packet, maxBlockTimeTicks, NULL);
}


///////////////////////////////////////////////////////////////////////////////
//	void i2c_WriteSingleRegister(I2C_HANDLE *i2cHandle,
//		uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t data)

void i2c_WriteSingleRegister(I2C_HANDLE *i2cHandle,
		uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t data)
{
	i2c_WriteRegisters(i2cHandle, i2cBaseAddress, i2cSubaddress, &data, 1);
}


///////////////////////////////////////////////////////////////////////////////
//  void i2c_ReadRegisters(I2C_HANDLE *i2cHandle,
//		uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t *buffer, uint8_t len)

void i2c_ReadRegisters(I2C_HANDLE *i2cHandle, 
		uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t *buffer, uint8_t len)
{
	portTickType maxBlockTimeTicks = (200UL / portTICK_RATE_MS);
	
	twi_packet_t p_packet;
	
	p_packet.chip			= i2cBaseAddress;
	p_packet.addr[0]		= i2cSubaddress;
	p_packet.addr_length	= 1;
	p_packet.buffer			= buffer;
	p_packet.length			= len;

	freertos_twi_read_packet_async (i2cHandle->twi, &p_packet, maxBlockTimeTicks, NULL);
}

///////////////////////////////////////////////////////////////////////////////
//  uint8_t i2c_ReadSingleRegister(I2C_HANDLE *i2cHandle,
//		uint8_t i2cBaseAddress, uint8_t i2cSubaddress)

uint8_t i2c_ReadSingleRegister(I2C_HANDLE *i2cHandle, 
		uint8_t i2cBaseAddress, uint8_t i2cSubaddress)
{
	portTickType maxBlockTimeTicks = (200UL / portTICK_RATE_MS);
	
	twi_packet_t p_packet;
	uint8_t value = 0;

	p_packet.chip			= i2cBaseAddress;
	p_packet.addr[0]		= i2cSubaddress;
	p_packet.addr_length	= 1;
	p_packet.buffer			= &value;
	p_packet.length			= 1;
	
	freertos_twi_read_packet_async (i2cHandle->twi, &p_packet, maxBlockTimeTicks, NULL);
	
	return value;
}


///////////////////////////////////////////////////////////////////////////////
//  uint8_t i2c_ClearBus(void)
//
// for documentation & background of this code, see: 
//
// https://spellfoundry.com/2020/06/25/reliable-embedded-systems-recovering-arduino-i2c-bus-lock-ups/
// http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html


/*
int I2C_ClearBus(int sdaPin, int sclPin) {
	#if defined(TWCR) && defined(TWEN)
	TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the sdaPin and sclPin pins directly
	#endif
	pinMode(sdaPin, INPUT_PULLUP); // Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
	pinMode(sclPin, INPUT_PULLUP);
	#ifndef ARDUINO_nRF52832_BARE_MODULE
	delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
	// skip this delay for bare module NRF5 chips as it will cause high power and prevent monitoring current consumption
	#endif
	// up of the DS3231 module to allow it to initialize properly,
	// but is also assists in reliable programming of FioV3 boards as it gives the
	// IDE a chance to start uploaded the program
	// before existing sketch confuses the IDE by sending Serial data.

	boolean SCL_LOW = (digitalRead(sclPin) == LOW); // Check is sclPin is Low.
	if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master.
		pinMode(sdaPin, INPUT_PULLUP); // Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
		pinMode(sclPin, INPUT_PULLUP);
		return 1; //I2C bus error. Could not clear sclPin clock line held low
	}

	boolean SDA_LOW = (digitalRead(sdaPin) == LOW);  // vi. Check sdaPin input.
	int clockCount = 20; // > 2x9 clock

	while (SDA_LOW && (clockCount > 0)) { //  vii. If sdaPin is Low,
		clockCount--;
		// Note: I2C bus is open collector so do NOT drive sclPin or sdaPin high.
		pinMode(sclPin, INPUT); // release sclPin pullup so that when made output it will be LOW
		pinMode(sclPin, OUTPUT); // then clock sclPin Low
		delayMicroseconds(10); //  for >5us
		pinMode(sclPin, INPUT); // release sclPin LOW
		pinMode(sclPin, INPUT_PULLUP); // turn on pullup resistors again
		// do not force high as slave may be holding it low for clock stretching.
		delayMicroseconds(10); //  for >5us
		// The >5us is so that even the slowest I2C devices are handled.
		SCL_LOW = (digitalRead(sclPin) == LOW); // Check if sclPin is Low.
		int counter = 20;
		while (SCL_LOW && (counter > 0)) {  //  loop waiting for sclPin to become High only wait 2sec.
			counter--;
			delay(100);
			SCL_LOW = (digitalRead(sclPin) == LOW);
		}
		if (SCL_LOW) { // still low after 2 sec error
			pinMode(sdaPin, INPUT_PULLUP); // Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
			pinMode(sclPin, INPUT_PULLUP);
			return 2; // I2C bus error. Could not clear. sclPin clock line held low by slave clock stretch for >2sec
		}
		SDA_LOW = (digitalRead(sdaPin) == LOW); //   and check sdaPin input again and loop
	}
	if (SDA_LOW) { // still low
		pinMode(sdaPin, INPUT_PULLUP); // Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
		pinMode(sclPin, INPUT_PULLUP);
		return 3; // I2C bus error. Could not clear. sdaPin data line held low
	}

	// else pull sdaPin line low for Start or Repeated Start
	pinMode(sdaPin, INPUT); // remove pullup.
	pinMode(sdaPin, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
	// When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
	/// A Repeat Start is a Start occurring after a Start with no intervening Stop.
	delayMicroseconds(10); // wait >5us
	pinMode(sdaPin, INPUT); // remove output low
	pinMode(sdaPin, INPUT_PULLUP); // and make sdaPin high i.e. send I2C STOP control.
	delayMicroseconds(10); // x. wait >5us
	pinMode(sdaPin, INPUT_PULLUP); // Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
	pinMode(sclPin, INPUT_PULLUP);
	return 0; // all ok
}

*/

///////////////////////////////////////////////////////////////////////////////
//  uint8_t i2c_ClearBus(void)
//
// resets the entire I2C bus & all devices. Otherwise, only power down 
// is an option... :-(

static uint8_t G_I2CClockRepairCount = 0;

uint8_t i2c_ClearBus(I2C_CHANNEL channel)
{
	due_pin_number_t sdaPin = 0;
	due_pin_number_t sclPin = 0;

	if( i2c_IsValidChannelNumber(channel) == false )
	{
		return 0xff;
	}

	// yes, physical channel 0 on the SAM3X8E is labeled channel 1 on the Arduino Due...:
	
	if (channel == I2C_CHANNEL_0)
	{
		sdaPin = PIN_SDA1;
		sclPin = PIN_SCL1;
	}
	else if (channel == I2C_CHANNEL_1)
	{
		sdaPin = PIN_SDA0;
		sclPin = PIN_SCL0;
	}
	
	// Make sdaPin (data) and sclPin (clock) pins Inputs with pull-up.
	dio_SetPinDirection(sdaPin, IOPORT_DIR_INPUT);
	dio_SetPinMode(sdaPin, IOPORT_MODE_PULLUP);

	dio_SetPinDirection(sclPin,	IOPORT_DIR_INPUT);
	dio_SetPinMode(sclPin, IOPORT_MODE_PULLUP);
	
	delay_ms(100);
	
	// If SCL is held low, Arduino cannot become the I2C master:
	
	bool isSCLLow = (dio_GetPin(sclPin) == false);
	if (isSCLLow) 
	{	
		// Make sdaPin (data) and sclPin (clock) pins inputs with pull-up.
		dio_SetPinDirection(sdaPin, IOPORT_DIR_INPUT);
		dio_SetPinMode(sdaPin,		IOPORT_MODE_PULLUP);

		dio_SetPinDirection(sclPin,	IOPORT_DIR_INPUT);
		dio_SetPinMode(sclPin,		IOPORT_MODE_PULLUP);

		// I2C bus error. Could not clear sclPin, clock line held low	
		return I2C_ERR_CLK_LOW;		
	}
		
	bool isSDALow = (dio_GetPin(sdaPin) == false);

	// if SCL is high and SDA is high, bus is ok:
	
	if ((isSCLLow == false) && (isSDALow == false))
	{
		return I2C_BUS_OK;
	}
	
	int clockCount = 40; // > 2 x 9 clock, see spec of I2C bus

	//  if sdaPin is Low, send dummy clock pulses to the slave:
	
	while (isSDALow && (clockCount > 0)) 
	{ 
		clockCount--;
		
		// Note: I2C bus is open collector so do NOT drive sclPin or sdaPin high.
		
		dio_SetPinDirection(sclPin,	IOPORT_DIR_INPUT);
		dio_SetPinMode(sclPin, 0); // release sclPin pull-up so that when made output it will be LOW

		dio_SetPinDirection(sclPin, IOPORT_DIR_OUTPUT); // then clock sclPin Low
		
		delay_us(20); //  for > 5us
		
		dio_SetPinDirection(sclPin, IOPORT_DIR_INPUT);	 // release sclPin LOW
		dio_SetPinMode(sclPin,		IOPORT_MODE_PULLUP); // turn on pull-up resistors again
		
		// do not force high as slave may be holding it low for clock stretching.
		// the > 5us is so that even the slowest I2C devices are handled.
		
		delay_us(20); //  for >5us

		G_I2CClockRepairCount++;
		
		isSCLLow = (dio_GetPin(sclPin) == false); // Check if sclPin is Low.

		int counter = 20;
		while (isSCLLow && (counter > 0)) //  loop waiting for sclPin to become High only wait 2sec.
		{  
			counter--;
			delay_ms(100);
			isSCLLow = (dio_GetPin(sclPin) == false);
		}
		
		// still low after 2 sec == error:
		
		if (isSCLLow) 
		{ 
			// Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
			
			dio_SetPinDirection(sdaPin, IOPORT_DIR_INPUT);
			dio_SetPinMode(sdaPin,		IOPORT_MODE_PULLUP);

			dio_SetPinDirection(sclPin, IOPORT_DIR_INPUT);
			dio_SetPinMode(sclPin,		IOPORT_MODE_PULLUP);

			// I2C bus error, could not clear the bus. sclPin clock line held low 
			// by slave clock stretch for > 2sec
			
			return I2C_ERR_CLK_LOW_SLAVE; 
		}
		isSDALow = (dio_GetPin(sdaPin) == false); //   and check sdaPin input again and loop
	}	
	
	if (isSDALow) // still low
	{ 
		// Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
		
		dio_SetPinDirection(sdaPin, IOPORT_DIR_INPUT);
		dio_SetPinMode(sdaPin,		IOPORT_MODE_PULLUP);
	
		dio_SetPinDirection(sclPin, IOPORT_DIR_INPUT);
		dio_SetPinMode(sclPin,		IOPORT_MODE_PULLUP);

		// I2C bus error. Could not clear. sdaPin data line held low.
		
		return I2C_ERR_SDA_LOW; 
	}

	// else pull sdaPin line low for Start or Repeated Start

	dio_SetPinDirection(sdaPin, IOPORT_DIR_INPUT);
	dio_SetPinMode(sdaPin, 0);	// remove pull-up
	
	// and then make it LOW i.e. send an I2C Start or Repeated start control.
	dio_SetPinDirection(sdaPin, IOPORT_DIR_OUTPUT);

	// When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
	/// A Repeat Start is a Start occurring after a Start with no intervening Stop.

	delay_us(20); // wait >5us
	
	dio_SetPinDirection(sdaPin, IOPORT_DIR_INPUT); // remove output low
	dio_SetPinMode(sdaPin,		IOPORT_MODE_PULLUP); // and make sdaPin high i.e. send I2C STOP control.
	
	delay_us(20); // wait > 5us
	
	// Make sdaPin (data) and sclPin (clock) pins Inputs with pullup.
	
	dio_SetPinDirection(sdaPin, IOPORT_DIR_INPUT);
	dio_SetPinMode(sdaPin,		IOPORT_MODE_PULLUP);

	dio_SetPinDirection(sclPin, IOPORT_DIR_INPUT);
	dio_SetPinMode(sclPin,		IOPORT_MODE_PULLUP);
	
	return I2C_BUS_REPAIRED; // repaired, I2C bus OK now
}


///////////////////////////////////////////////////////////////////////////////
//  uint8_t i2c_GetRepairCount(void)

uint8_t i2c_GetRepairCount(void)
{
	return G_I2CClockRepairCount;
}

///////////////////////////////////////////////////////////////////////////////
//  const char *i2c_GetErrorMessage(uint8_t i2cErrorCode)

const char *i2c_GetErrorMessage(uint8_t i2cErrorCode)
{
	const char *msg = NULL;
	
	if (i2cErrorCode == I2C_BUS_OK)
	{
		msg = "I2C - BUS OK";
	}
	else if (i2cErrorCode == I2C_BUS_REPAIRED)
	{
		msg = "I2C - BUS REPAIRED";
	}
	else if (i2cErrorCode == I2C_ERR_CLK_LOW)
	{
		msg = "I2C - CLK LINE LOW";
	}
	else if (i2cErrorCode == I2C_ERR_SDA_LOW)
	{
		msg = "I2C - SDA LINE LOW";
	}
	else if (i2cErrorCode == I2C_ERR_CLK_LOW_SLAVE)
	{
		msg = "I2C - SLAVE CLK LINE LOW";
	}
	else
	{
		msg = "I2C - INVALID ERROR CODE";
	}
	
	return msg;
}
