/*
 * AccelMMA8452Lib.c
 *
 * Created: 29-1-2023 08:18:54
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
#include "AccelMMA8452Lib.h"
#include "vPrintString.h"

///////////////////////////////////////////////////////////////////////////////
// bool accel_Init(I2C_CHANNEL i2cChannelNumber)

bool accel_Init(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	bool initOK			  = true;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		accel_GotoStandby(i2cChannelNumber, true);
		// Reset then switch to active mode with 100Hz output:
		//i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, REG1_STANDBY);		// standby
		//i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, REG1_RESET);			// reset
		//i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG0, REG0_FS_1000DPS);	// set sensitivity
		//i2c_WriteSingleRegister(i2cHandle, FXAS21002C_ADDRESS, GYRO_CTRL_REG1, REG1_ODR_100	| REG1_ACTIVE);	// 100 Hz &  go to active state
		accel_GotoStandby(i2cChannelNumber, false);

	}
	else
	{
		initOK = false;
	}

	return initOK;
}

///////////////////////////////////////////////////////////////////////////////
// bool accel_SetFullScale(I2C_CHANNEL i2cChannelNumber, uint8_t fullScale)
// only value of 2, 4 or 8 for fullScale allowed!!
// device must be in Standby mode

bool accel_SetFullScale(I2C_CHANNEL i2cChannelNumber, uint8_t fullScale)
{
	I2C_HANDLE *i2cHandle = NULL;
	accelScale_t scale	= SCALE_2G;
	bool resultOK = false;
	
	switch (fullScale)
	{
		case 2:
		scale = SCALE_2G;
		resultOK = true;
		break;
		
		case 4:
		scale = SCALE_4G;
		resultOK = true;
		break;
		
		case 8:
		scale = SCALE_8G;
		resultOK = true;
		break;
		
		default:
		resultOK = false;
		break;
	}
	
	if (resultOK)
	{
		i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
		i2c_WriteSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_XYZ_DATA_CFG, scale);
	}
	
	return resultOK;
}


///////////////////////////////////////////////////////////////////////////////
// uint8_t accel_GetDeviceId(I2C_CHANNEL i2cChannelNumber)

uint8_t accel_GetDeviceId(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t accelDeviceId = 0;		// 0 = invalid id
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);

	if (i2cHandle != NULL)
	{
		accelDeviceId = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_WHO_AM_I);
	}

	return accelDeviceId;
}

///////////////////////////////////////////////////////////////////////////////
// bool accel_EnableOrientationDetection(I2C_CHANNEL i2cChannelNumber)
// device must be in Standby mode to enable / disable orientation detection!

bool accel_EnableOrientationDetection(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t plConfig	  = 0;
	uint8_t debounceCount = 80;		// == 100ms @800Hz ODR
	bool	resultOK	  = true;
		
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		plConfig = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_PL_CFG);		
		plConfig = plConfig | PL_CFG_PL_EN;
		i2c_WriteSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_PL_CFG,   plConfig);
		i2c_WriteSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_PL_COUNT, debounceCount);
	}
	else
	{
		resultOK = false;
	}
	
	return resultOK;
}


///////////////////////////////////////////////////////////////////////////////
// void accel_EnableOrientationDetection(I2C_CHANNEL i2cChannelNumber)

bool accel_OrientationChanged(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t	plStatus = 0;
	bool orientationChanged = false;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{	
		plStatus = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_PL_STATUS);
		orientationChanged = ((plStatus & PL_STATUS_NEWLP) != 0);
	}
	else
	{
		orientationChanged = false;
	}
	
	return orientationChanged;
}


///////////////////////////////////////////////////////////////////////////////
// bool accel_GetOrientation(I2C_CHANNEL i2cChannelNumber, 
//							 uint8_t *plOrientation, bool *isBack)

bool accel_GetOrientation(I2C_CHANNEL i2cChannelNumber, uint8_t *plOrientation, bool *isBack)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t	plStatus = 0;
	bool resultOK	 = true;
	
	*plOrientation = ORIENTATION_UNKNOWN;
	*isBack		   = false;

	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		plStatus = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_PL_STATUS);
		if ((plStatus & PL_STATUS_LOCKOUT) == 0)
		{
			*plOrientation = (plStatus & PL_STATUS_ORIENTATION) >> 1;
		}
		else
		{
			*plOrientation = ORIENTATION_Z_LOCKOUT;
		}
		*isBack = (plStatus & PL_STATUS_BACK);
	}
	else
	{
		resultOK = false;
	}
	
	return resultOK;
}


///////////////////////////////////////////////////////////////////////////////
// Text strings for orientations

static const char *PLOrientationStrings[] =
{
	"PORTRAIT UP (0)",
	"PORTRAIT DOWN (1)",
	"LANDSCAPE RIGHT (2)",
	"LANDSCAPE LEFT (3)",
	"FLAT (4)"				// same as lock-out
};

static const char *FrontBackStrings[] =
{
	"FRONT",
	"BACK",
};


///////////////////////////////////////////////////////////////////////////////
// void accel_PrintOrientationString(char *buffer, uint16_t bufSize, 
//									 uint8_t plOrientation, bool isBack)

void accel_PrintOrientationString(char *buffer, uint16_t bufSize, 
								  uint8_t plOrientation, bool isBack)
{
	uint8_t stringIndex = isBack ? 1 : 0;
	
	snprintf(buffer, bufSize, "%s - %s", 
			PLOrientationStrings[plOrientation], FrontBackStrings[stringIndex]);
}


///////////////////////////////////////////////////////////////////////////////
// uint8_t accel_GetSystemMode(I2C_CHANNEL i2cChannelNumber)

uint8_t accel_GetSystemMode(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t	systemMode	  = SYSMOD_UNKNOWN;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		systemMode = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_SYSMOD);
	}
	
	return systemMode;	
}


///////////////////////////////////////////////////////////////////////////////
// bool accel_GotoStandby(I2C_CHANNEL i2cChannelNumber, bool standby)

bool accel_GotoStandby(I2C_CHANNEL i2cChannelNumber, bool standby)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t reg1Value	  = 0;
	bool	resultOK	  = true;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		reg1Value = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG1);
		if (standby)
		{
			reg1Value = reg1Value & ~CTRL_REG1_ACTIVE;	// goto standby state
		}
		else
		{
			reg1Value = reg1Value | CTRL_REG1_ACTIVE;	// else go to active state
		}
		i2c_WriteSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG1, reg1Value);
	}
	else
	{
		resultOK = false;
	}

	return resultOK;
}

///////////////////////////////////////////////////////////////////////////////
// void accel_EnableOrientationChangeInterrupt(I2C_CHANNEL i2cChannelNumber)
// device must be in Standby mode!

void accel_EnableOrientationChangeInterrupt(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t ctrlReg4Value = 0;
	uint8_t ctrlReg5Value = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		// map orientation change interrupt to device pin INT1
		ctrlReg5Value  = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG5);
		ctrlReg5Value |= INT_CFG_LNDPRT;
		i2c_WriteSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG5, ctrlReg5Value);

		// enable orientation interrupt changes
		ctrlReg4Value  = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG4);
		ctrlReg4Value |= INT_EN_LNDPRT;
		i2c_WriteSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG4, ctrlReg4Value);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void accel_DisableOrientationChangeInterrupt(I2C_CHANNEL i2cChannelNumber)
// device must be in Standby mode!

void accel_DisableOrientationChangeInterrupt(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t ctrlReg4Value = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		// disable orientation interrupt changes
		ctrlReg4Value  = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG4);
		ctrlReg4Value &= ~INT_EN_LNDPRT;
		i2c_WriteSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_CTRL_REG4, ctrlReg4Value);
	}
}


///////////////////////////////////////////////////////////////////////////////
// uint8_t accel_GetInterruptSource(I2C_CHANNEL i2cChannelNumber)

uint8_t accel_GetInterruptSource(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t interruptSource = 0xFF;	// assume invalid source...
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		interruptSource = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_INT_SOURCE);
	}
	
	return interruptSource;
}

//////////////////////////////////////////////////////////////////////////////
// bool accel_IsDataAvailable(I2C_CHANNEL i2cChannelNumber)

bool accel_IsDataAvailable(I2C_CHANNEL i2cChannelNumber)
{
	I2C_HANDLE *i2cHandle = NULL;
	bool	available	  = false;
	uint8_t status		  = 0;
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);
	
	if (i2cHandle != NULL)
	{
		status = i2c_ReadSingleRegister(i2cHandle, MMA8452_ADDRESS, ACCEL_STATUS);
		available = ((status & STATUS_ZYXDR) != 0);
	}
	else
	{
		available = false;
	}
	
	return available;
}


///////////////////////////////////////////////////////////////////////////////
// bool accel_ReadRawAxisData(I2C_CHANNEL i2cChannelNumber, accelRawData_t *rawAxisData)

bool accel_ReadRawAxisData(I2C_CHANNEL i2cChannelNumber, accelRawData_t *rawAxisData)
{
	I2C_HANDLE *i2cHandle = NULL;
	uint8_t accelStatus = 0;
	bool resultOK = false;

	// consecutive read of 6 data registers
	const uint8_t nBytesToRead = sizeof(accelRawData_t) + 1;
	uint8_t buffer[nBytesToRead];
	
	i2cHandle = i2c_GetChannelHandle(i2cChannelNumber);

	// set all data to zero
	memset(rawAxisData, 0, sizeof(accelRawData_t));
	memset(buffer,		0, sizeof(buffer));
	
	if (i2cHandle != NULL)
	{
			// read status register & 6 data registers
			i2c_ReadRegisters(i2cHandle, MMA8452_ADDRESS, ACCEL_STATUS, buffer, nBytesToRead);
			accelStatus = buffer[0];
			if ((accelStatus & STATUS_ZYXDR) != 0)	//	new data available
			{
				// Shift 8 byte values to create properly formed 12 bit integers
				rawAxisData->x = (int16_t)((buffer[1] << 8) | buffer[2]) >> 4;
				rawAxisData->y = (int16_t)((buffer[3] << 8) | buffer[4]) >> 4;
				rawAxisData->z = (int16_t)((buffer[5] << 8) | buffer[6]) >> 4;
				resultOK = true;
			}
			else // no valid data available
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
// void accel_CalculateXYZAcceleration(	accelRawData_t *rawAxisData, 
//										uint8_t fullScale, 
//										accelFloatData_t *accelFloatData)

void accel_CalculateXYZAcceleration(accelRawData_t *rawAxisData, uint8_t fullScale, accelFloatData_t *accelFloatData)
{
	double factor = 1 << 11;
	
	accelFloatData->accelX = ((rawAxisData->x) / factor) * fullScale;
	accelFloatData->accelY = ((rawAxisData->y) / factor) * fullScale;
	accelFloatData->accelZ = ((rawAxisData->z) / factor) * fullScale;
}
