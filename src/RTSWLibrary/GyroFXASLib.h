/*
 * GyroFXASLib.h
 *
 * Created: 18-10-2022 23:06:18
 *  Author: Roel Smeets
 */ 


#ifndef GYROFXASLIB_H_
#define GYROFXASLIB_H_

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "bits.h"
#include "I2CLib.h"

///////////////////////////////////////////////////////////////////////////////
// #defines

// 7-bit I2C default base address for this sensor
#define FXAS21002C_ADDRESS	(0x21)

// Device ID for this sensor (used as a sanity check during init)
#define FXAS21002C_ID		(0xD7)

///////////////////////////////////////////////////////////////////////////////
// gyroscope sensitivities, Table 35 of datasheet

// Gyroscope sensitivity at 250dps
#define GYRO_SENSITIVITY_250DPS (0.0078125F) 

// Gyroscope sensitivity at 500dps
#define GYRO_SENSITIVITY_500DPS (0.015625F)

// Gyroscope sensitivity at 1000dps
#define GYRO_SENSITIVITY_1000DPS (0.03125F)

// Gyroscope sensitivity at 2000dps
#define GYRO_SENSITIVITY_2000DPS (0.0625F)

///////////////////////////////////////////////////////////////////////////////
// valid gyroscope output data rate values(ODR)
    
#define GYRO_ODR_800HZ	(800.0f)	// 800Hz
#define GYRO_ODR_400HZ	(400.0f)	// 400Hz
#define GYRO_ODR_200HZ	(200.0f)	// 200Hz/
#define GYRO_ODR_100HZ	(100.0f)	// 100Hz/
#define GYRO_ODR_50HZ	(50.0f)		// 50Hz/
#define GYRO_ODR_25HZ	(25.0f)		// 25Hz
#define GYRO_ODR_12_5HZ (12.5f)		// 12.5Hz


///////////////////////////////////////////////////////////////////////////////
// Register offsets from base address

typedef enum 
{
  GYRO_STATUS		= 0x00,
  GYRO_OUT_X_MSB	= 0x01,
  GYRO_OUT_X_LSB	= 0x02,
  GYRO_OUT_Y_MSB	= 0x03,
  GYRO_OUT_Y_LSB	= 0x04,
  GYRO_OUT_Z_MSB	= 0x05,
  GYRO_OUT_Z_LSB	= 0x06,
  GYRO_WHO_AM_I		= 0x0C,
  GYRO_CTRL_REG0	= 0x0D,
  GYRO_TEMPERATURE	= 0x12,
  GYRO_CTRL_REG1	= 0x13,
  GYRO_CTRL_REG2	= 0x14,
} gyroRegisters_t;


///////////////////////////////////////////////////////////////////////////////
// DR_STATUS bit definitions

#define DRSTAT_ZYXOW			BIT_7
#define DRSTAT_ZOW				BIT_6
#define DRSTAT_YOW				BIT_5
#define DRSTAT_XOW				BIT_4
#define DRSTAT_ZYXDR			BIT_3
#define DRSTAT_ZDR				BIT_2
#define DRSTAT_YDR				BIT_1
#define DRSTAT_YDR				BIT_1
#define DRSTAT_XDR				BIT_0

///////////////////////////////////////////////////////////////////////////////
// CTRL_REG0 bit definitions

#define REG0_FS_2000DPS			0
#define REG0_FS_1000DPS			1
#define REG0_FS_500DPS			2
#define REG0_FS_250DPS			3

#define REG0_HIPASS_ENABLE		BIT_2

#define REG0_CUTOFF_SELECT_0	(0 << 3)
#define REG0_CUTOFF_SELECT_1	(1 << 3)
#define REG0_CUTOFF_SELECT_2	(2 << 3)

#define REG0_BW_SELECT_0		(0 << 6)
#define REG0_BW_SELECT_1		(1 << 6)
#define REG0_BW_SELECT_2		(2 << 6)
#define REG0_BW_SELECT_3		(3 << 6)


///////////////////////////////////////////////////////////////////////////////
// CTRL_REG1 bit definitions 

#define REG1_READY		BIT_0
#define REG1_ACTIVE		BIT_1
#define REG1_SELFTEST	BIT_5
#define REG1_RESET		BIT_6

#define REG1_ODR_800	(0 << 2)
#define REG1_ODR_400	(1 << 2)
#define REG1_ODR_200	(2 << 2)
#define REG1_ODR_100	(3 << 2)
#define REG1_ODR_50		(4 << 2)
#define REG1_ODR_25		(5 << 2)
#define REG1_ODR_12_5	(6 << 2)

#define REG1_STANDBY	0x00


///////////////////////////////////////////////////////////////////////////////
// CTRL_REG2 bit definitions

#define REG2_INT_DATAREADY		BIT_3
#define REG2_INT_ENABLE			BIT_2
#define REG2_INT_POLARITY_HIGH	BIT_1



///////////////////////////////////////////////////////////////////////////////
// OPTIONAL SPEED SETTINGS to define valid gyroscope range values

typedef enum 
{
  GYRO_RANGE_250DPS		= 250,	// 250 dps
  GYRO_RANGE_500DPS		= 500,  // 500 dps
  GYRO_RANGE_1000DPS	= 1000, // 1000 dps
  GYRO_RANGE_2000DPS	= 2000  // 2000 dps
} gyroRange_t;


///////////////////////////////////////////////////////////////////////////////
// Raw gyroscope data type
//
// Structure to store a single raw (integer-based) gyroscope vector

typedef struct 
{
  int16_t x;	// raw int16_t value for the x axis
  int16_t y;	// raw int16_t value for the y axis
  int16_t z;	// raw int16_t value for the z axis
} gyroRawData_t;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

bool	gyro_Init(I2C_CHANNEL i2cChannelNumber);

uint8_t gyro_GetDeviceId(I2C_CHANNEL i2cChannelNumber);
int8_t  gyro_GetTemperature(I2C_CHANNEL i2cChannelNumber);

bool	gyro_GotoStandby(I2C_CHANNEL i2cChannelNumber, bool standby);
bool	gyro_ReadRawAxisData(I2C_CHANNEL i2cChannelNumber, gyroRawData_t *rawAxisData);
bool	gyro_SetRange(I2C_CHANNEL i2cChannelNumber, gyroRange_t range);
bool	gyro_SelfTest(I2C_CHANNEL i2cChannelNumber, bool enableSelftest);
bool	gyro_IsDataAvailable(I2C_CHANNEL i2cChannelNumber);

void	gyro_EnableInterrupt(I2C_CHANNEL i2cChannelNumber);
void	gyro_DisableInterrupt(I2C_CHANNEL i2cChannelNumber);

#endif /* GYROFXASLIB_H_ */
