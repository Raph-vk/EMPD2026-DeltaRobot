/*
 * GyroFXASLib.c
 *
 * Created: 18-10-2022 23:02:17
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "I2CLib.h"
#include "GyroFXASLib.h"
#include "vPrintString.h"


///////////////////////////////////////////////////////////////////////////////
// bool gyro_Init(I2C_CHANNEL i2cChannelNumber)

bool gyro_Init(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	bool initOK			  = true;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		// Reset then switch to active mode with 100Hz output:
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, REG1_STANDBY);		// standby
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, REG1_RESET);			// reset
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG0, REG0_FS_1000DPS);	// set sensitivity
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, REG1_ODR_100	| REG1_ACTIVE);	// 100 Hz &  go to active state
		delay_ms(100);	// 60ms + 1/ODR
	}
	else
	{
		initOK = false;
	}

	return initOK;
}

///////////////////////////////////////////////////////////////////////////////
// uint8_t gyro_GetDeviceId(I2C_CHANNEL i2cChannelNumber)

uint8_t gyro_GetDeviceId(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t gyroDeviceId  = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		gyroDeviceId = i2c_ReadSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_WHO_AM_I);
	}

	return gyroDeviceId;
}


///////////////////////////////////////////////////////////////////////////////
// int8_t gyro_GetTemperature(I2C_CHANNEL i2cChannelNumber)

int8_t gyro_GetTemperature(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	int8_t temperature	  = -78;	// some random unlikely value
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		temperature = i2c_ReadSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_TEMPERATURE);
	}

	return temperature;
}

///////////////////////////////////////////////////////////////////////////////
// bool gyro_GotoStandby(I2C_CHANNEL i2cChannelNumber, bool standby)

bool gyro_GotoStandby(I2C_CHANNEL i2cChannelNumber, bool standby)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t reg1Value	  = 0;
	bool	resultOK	  = true;
		
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
		
	if (i2cHandle != NULL)
	{
		if (standby)
		{
			reg1Value = REG1_STANDBY;	// goto standby state
		}
		else
		{
			reg1Value = REG1_ACTIVE;	// else go ta active state
		}
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, reg1Value);	
	}
	else
	{
		resultOK = false;
	}

	delay_ms(100);	// 60ms + 1/ODR when switching mode from active to standby v.v., see specs

	return resultOK;
}

///////////////////////////////////////////////////////////////////////////////
// bool gyro_ReadRawAxisData(I2C_CHANNEL i2cChannelNumber, gyroRawData_t *rawAxisData)

bool gyro_ReadRawAxisData(I2C_CHANNEL i2cChannelNumber, gyroRawData_t *rawAxisData)
{
	I2C_HANDLE *i2cHandle = NULL;
	bool resultOK = false;

	// consecutive read of status register & 6 data registers
	const uint8_t nBytesToRead = sizeof(gyroRawData_t) + 1;
	uint8_t buffer[nBytesToRead];	
			
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);

	// set all data to zero
	memset(rawAxisData, 0, sizeof(gyroRawData_t));
	memset(buffer,		0, sizeof(buffer));
	
	if (i2cHandle != NULL)
	{
		if (gyro_IsDataAvailable(i2cChannelNumber))
		{
			// read status register & 6 data registers
			i2c_ReadRegisters(i2cHandle, FXAS21002C_ADDRESS, GYRO_STATUS, buffer, nBytesToRead);
			
			// Shift 8 byte values to create properly formed 16 bit integers
			rawAxisData->x = (int16_t)((buffer[1] << 8) | buffer[2]);
			rawAxisData->y = (int16_t)((buffer[3] << 8) | buffer[4]);
			rawAxisData->z = (int16_t)((buffer[5] << 8) | buffer[6]);
			
			resultOK = true;
		}
		else
		{
			resultOK = false;
		}
	}
	else
	{
		resultOK = false;
	}
	
	return resultOK;
}

///////////////////////////////////////////////////////////////////////////////
// bool gyro_SetRange(I2C_CHANNEL i2cChannelNumber, gyroRange_t range)

bool gyro_SetRange(I2C_CHANNEL i2cChannelNumber, gyroRange_t range)
{
	I2C_HANDLE *i2cHandle = NULL;
	bool resultOK = true;
	uint8_t ctrlReg0	= 0;
	uint8_t rangeValue	= 0;

	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		if (range == GYRO_RANGE_2000DPS)
		{
			rangeValue = REG0_FS_2000DPS;
		}
		else if (range == GYRO_RANGE_1000DPS)
		{
			rangeValue = REG0_FS_1000DPS;
		}
		else if (range == GYRO_RANGE_500DPS)
		{
			rangeValue = REG0_FS_500DPS;
		}
		else if (range == GYRO_RANGE_250DPS)
		{
			rangeValue = REG0_FS_250DPS;
		}
		
		gyro_GotoStandby(i2cChannelNumber, true);
		ctrlReg0 = i2c_ReadSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG0);
		ctrlReg0 = ctrlReg0 & (~(BIT_1 | BIT_0));
		ctrlReg0 = ctrlReg0 | rangeValue;
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG0, ctrlReg0);
		
		gyro_GotoStandby(i2cChannelNumber, false);	// go to active mode again
	}
	else
	{
		resultOK = false;
	}	
	
	return resultOK;
}

///////////////////////////////////////////////////////////////////////////////
// void gyro_EnableInterrupt(I2C_CHANNEL i2cChannelNumber)

void gyro_EnableInterrupt(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t ctrlReg2Value = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
		
	if (i2cHandle != NULL)
	{
		ctrlReg2Value  = i2c_ReadSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG2);
		ctrlReg2Value |= (REG2_INT_DATAREADY | REG2_INT_ENABLE | REG2_INT_POLARITY_HIGH);
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG2, ctrlReg2Value);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void gyro_DisableInterrupt(I2C_CHANNEL i2cChannelNumber)

void gyro_DisableInterrupt(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t ctrlReg2Value = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		ctrlReg2Value  = i2c_ReadSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG2);
		ctrlReg2Value &= ~(REG2_INT_ENABLE);
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG2, ctrlReg2Value);
	}
}

///////////////////////////////////////////////////////////////////////////////
// bool gyro_SelfTest(I2C_CHANNEL i2cChannelNumber, bool enableSelftest)

bool gyro_SelfTest(I2C_CHANNEL i2cChannelNumber, bool enableSelftest)
{
	I2C_HANDLE *i2cHandle = NULL;
	bool	resultOK	  = true;
	uint8_t reg1Value	  = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		gyro_GotoStandby(i2cChannelNumber, true);
		
		// clear range bits = max. range, required for selftest, see table 5 in manual
		resultOK = gyro_SetRange(i2cChannelNumber, GYRO_RANGE_2000DPS);
		
		reg1Value = i2c_ReadSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1);
		if (enableSelftest)
		{
			reg1Value = reg1Value | REG1_SELFTEST;
		}
		else
		{
			reg1Value = reg1Value & (~REG1_SELFTEST);
		}
		i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, reg1Value);
		
		gyro_GotoStandby(i2cChannelNumber, false);	// go to active mode again
	}
	else
	{
		resultOK = false;
	}
	
	return resultOK;
}

//////////////////////////////////////////////////////////////////////////////
// bool gyro_IsDataAvailable(I2C_CHANNEL i2cChannelNumber)

bool gyro_IsDataAvailable(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	bool	available	  = false;
	uint8_t status		  = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		status = i2c_ReadSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_STATUS);
		available = ((status & DRSTAT_ZYXDR) != 0);
	}
	else
	{
		available = false;
	}
	
	return (available);	
}
