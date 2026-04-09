/*
 * I2CLib.h
 *
 * Created: 17-10-2022 17:17:24
 *  Author: Roel Smeets
 */ 


#ifndef I2CLIB_H_
#define I2CLIB_H_

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>

///////////////////////////////////////////////////////////////////////////////
// #define's

#define I2C_RECEIVE_BUFFER_SIZE		100
#define I2C_FAST_MODE_SPEED			400000


#define I2C_BUS_OK					0
#define I2C_BUS_REPAIRED			1
#define I2C_ERR_CLK_LOW				2
#define I2C_ERR_SDA_LOW				3
#define I2C_ERR_CLK_LOW_SLAVE		4


///////////////////////////////////////////////////////////////////////////////
// type definitions

typedef enum
{
	I2C_CHANNEL_0,
	I2C_CHANNEL_1,
	I2C_N_CHANNELS
} I2C_CHANNEL;


typedef struct
{
	uint32_t sda;
	uint32_t scl;
	uint32_t interrupt;
} I2C_IO_DESC;


typedef struct
{
	I2C_IO_DESC	ioDesc;
	Twi			*twi;
	freertos_peripheral_options_t i2c_options;
} I2C_HANDLE;


///////////////////////////////////////////////////////////////////////////////
// function prototypes

bool i2c_Init(void);
uint8_t i2c_ClearBus(I2C_CHANNEL channel);

bool i2c_InitChannel(I2C_CHANNEL channel);
I2C_HANDLE *i2c_GetChannelHandle(I2C_CHANNEL i2cChannelNumber);
bool i2c_IsValidChannelNumber(I2C_CHANNEL i2cChannelNumber);

void i2c_WriteRegisters(I2C_HANDLE *i2cHandle, uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t *buffer, uint8_t len);
void i2c_WriteSingleRegister(I2C_HANDLE *i2cHandle, uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t data);
void i2c_ReadRegisters(I2C_HANDLE *i2cHandle, uint8_t i2cBaseAddress, uint8_t i2cSubaddress, uint8_t *buffer, uint8_t len);
uint8_t i2c_ReadSingleRegister(I2C_HANDLE *i2cHandle, uint8_t i2cBaseAddress, uint8_t i2cSubaddress);


///////////////////////////////////////////////////////////////////////////////
// functions for diagnosing I2C bus lockup
// errorCode is return value of function i2c_ClearBus()

uint8_t i2c_GetRepairCount(void);
const char *i2c_GetErrorMessage(uint8_t i2cErrorCode);


#endif /* I2CLIB_H_ */