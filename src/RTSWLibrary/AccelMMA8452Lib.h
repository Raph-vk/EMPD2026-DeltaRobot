/*
 * AccellMMA8452Lib.h
 *
 * Created: 29-1-2023 08:19:17
 *  Author: Roel Smeets
 */ 


#ifndef ACCELLMMA8452LIB_H_
#define ACCELLMMA8452LIB_H_

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "bits.h"
#include "I2CLib.h"

///////////////////////////////////////////////////////////////////////////////
// #defines

// 7-bit I2C default base address for this device
#define MMA8452_ADDRESS	(0x1C)		// if SA0 = 0
//#define MMA8452_ADDRESS	(0x1D)	// if SA0 = 1


// Device ID for the MMA8452 sensor (used as a sanity check during init)
#define MMA8452_ID		(0x2A)


///////////////////////////////////////////////////////////////////////////////
// Register offsets from base address

typedef enum
{
	ACCEL_STATUS			= 0x00,
	ACCEL_OUT_X_MSB			= 0x01,
	ACCEL_OUT_X_LSB			= 0x02,
	ACCEL_OUT_Y_MSB			= 0x03,
	ACCEL_OUT_Y_LSB			= 0x04,
	ACCEL_OUT_Z_MSB			= 0x05,
	ACCEL_OUT_Z_LSB			= 0x06,
	
	ACCEL_SYSMOD			= 0x0B,
	ACCEL_INT_SOURCE		= 0x0C,
	ACCEL_WHO_AM_I			= 0x0D,
	ACCEL_XYZ_DATA_CFG		= 0x0E,
	ACCEL_HP_FILTER_CUTOFF	= 0x0F,
	ACCEL_PL_STATUS			= 0x10,
	ACCEL_PL_CFG			= 0x11,
	ACCEL_PL_COUNT			= 0x12,
	ACCEL_PL_BF_ZCOMP		= 0x13,
	ACCEL_P_L_THS_REG		= 0x14,
	ACCEL_FF_MT_CFG			= 0x15,
	ACCEL_FF_MT_SRC			= 0x16,
	ACCEL_FF_MT_THS			= 0x17,
	ACCEL_FF_MT_COUNT		= 0x18,
	
	ACCEL_TRANSIENT_CFG		= 0x1D,
	ACCEL_TRANSIENT_SRC		= 0x1E,
	ACCEL_TRANSIENT_THS		= 0x1F,
	ACCEL_TRANSIENT_COUNT	= 0x20,
	ACCEL_PULSE_CFG			= 0x21,
	ACCEL_PULSE_SRC			= 0x22,
	ACCEL_PULSE_THSX		= 0x23,
	ACCEL_PULSE_THSY		= 0x24,
	ACCEL_PULSE_THSZ		= 0x25,
	ACCEL_PULSE_TMLT		= 0x26,
	ACCEL_PULSE_LTCY		= 0x27,
	ACCEL_PULSE_WIND		= 0x28,
	ACCEL_ASLP_COUNT		= 0x29,
	ACCEL_CTRL_REG1			= 0x2A,
	ACCEL_CTRL_REG2			= 0x2B,
	ACCEL_CTRL_REG3			= 0x2C,
	ACCEL_CTRL_REG4			= 0x2D,
	ACCEL_CTRL_REG5			= 0x2E,
	ACCEL_OFF_X				= 0x2F,
	ACCEL_OFF_Y				= 0x30,
	ACCEL_OFF_Z				= 0x31,
	
} accelRegisters_t;


///////////////////////////////////////////////////////////////////////////////
// definitions for STATUS register

#define STATUS_ZYXDR		BIT_3


///////////////////////////////////////////////////////////////////////////////
// definitions for SYSMOD register

#define SYSMOD_STANDBY		0
#define SYSMOD_WAKE			1
#define SYSMOD_SLEEP		2
#define SYSMOD_UNKNOWN		0xFF


///////////////////////////////////////////////////////////////////////////////
// bit definitions for PL_STATUS register

#define PL_STATUS_NEWLP			BIT_7
#define PL_STATUS_LOCKOUT		BIT_6
#define PL_STATUS_ORIENTATION	(BIT_2 | BIT_1)
#define PL_STATUS_BACK			BIT_0

///////////////////////////////////////////////////////////////////////////////
// orientation definitions for PL_STATUS register

#define ORIENTATION_PORTRAIT_UP			0
#define ORIENTATION_PORTRAIT_DOWN		1
#define ORIENTATION_LANDSCAPE_UP		2
#define ORIENTATION_LANDSCAPE_DOWN		3
#define ORIENTATION_Z_LOCKOUT			4		// == flat

#define ORIENTATION_FRONT				0
#define ORIENTATION_BACK				1

#define ORIENTATION_UNKNOWN				0xFF

///////////////////////////////////////////////////////////////////////////////
// bit definitions for PL_CFG register

#define PL_CFG_PL_EN		BIT_6

///////////////////////////////////////////////////////////////////////////////
// bit definitions for CTRL_REG1 register

#define CTRL_REG1_ACTIVE	BIT_0


///////////////////////////////////////////////////////////////////////////////
// bit definitions for CTRL_REG4 register (enable / disable interrupts)

#define INT_EN_LNDPRT		BIT_4

///////////////////////////////////////////////////////////////////////////////
// bit definitions for CTRL_REG5 register (interrupt routing to pin 1 or 2)

#define INT_CFG_LNDPRT		BIT_4	// 0: INTPIN = 2, 1: INTPIN = 1


///////////////////////////////////////////////////////////////////////////////
// Full scale selection values FS1..FS0 bits for XYZ_DATA_CFG register

typedef enum
{	SCALE_2G = 0,		// Full Scale = 2g
	SCALE_4G = 1,		// Full Scale = 4g
	SCALE_8G = 2,		// Full Scale = 8g
} accelScale_t;
		
		
///////////////////////////////////////////////////////////////////////////////
// Raw acceleration data type
//
// Structure to store a single raw (integer-based) 3D acceleration vector

typedef struct
{
	int16_t x;	// raw int16_t value for the x axis
	int16_t y;	// raw int16_t value for the y axis
	int16_t z;	// raw int16_t value for the z axis
} accelRawData_t;


///////////////////////////////////////////////////////////////////////////////
// Acceleration data type, expressed in g's
//
// Structure to store a float type 3D acceleration vector

typedef struct
{
	float accelX;	// x acceleration
	float accelY;	// y acceleration
	float accelZ;	// z acceleration
} accelFloatData_t;


///////////////////////////////////////////////////////////////////////////////
// function prototypes

bool	accel_Init(I2C_CHANNEL i2cChannelNumber);
bool	accel_GotoStandby(I2C_CHANNEL i2cChannelNumber, bool standby);
bool	accel_EnableOrientationDetection(I2C_CHANNEL i2cChannelNumber);
bool	accel_OrientationChanged(I2C_CHANNEL i2cChannelNumber);
uint8_t accel_GetSystemMode(I2C_CHANNEL i2cChannelNumber);
uint8_t accel_GetDeviceId(I2C_CHANNEL i2cChannelNumber);
bool	accel_GetOrientation(I2C_CHANNEL i2cChannelNumber, 
							 uint8_t *plOrientation,
							 bool *isBack);
void accel_PrintOrientationString(char *buffer, uint16_t bufSize, 
								  uint8_t plOrientation, bool isBack);
void accel_EnableOrientationChangeInterrupt(I2C_CHANNEL i2cChannelNumber);
void accel_DisableOrientationChangeInterrupt(I2C_CHANNEL i2cChannelNumber);
uint8_t accel_GetInterruptSource(I2C_CHANNEL i2cChannelNumber);
bool accel_IsDataAvailable(I2C_CHANNEL i2cChannelNumber);
bool accel_ReadRawAxisData(I2C_CHANNEL i2cChannelNumber, accelRawData_t *rawAxisData);
bool accel_SetFullScale(I2C_CHANNEL i2cChannelNumber, uint8_t fullScale);
void accel_CalculateXYZAcceleration(accelRawData_t *rawAxisData, uint8_t fullScale, 
									accelFloatData_t *accelFloatData);
								  
#endif /* ACCELLMMA8452LIB_H_ */