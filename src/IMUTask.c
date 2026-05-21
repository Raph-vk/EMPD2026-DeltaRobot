
/*
 * IMUTask.c
 *
 * Created: 22/04/2026 08:38:01
 *  Author: raphv
 */ 
#include "IMUTask.h"
///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "I2CLib.h"
#include "vPrintString.h"



///////////////////////////////////////////////////////////////////////////////
// type definitions
typedef struct
{
	TickType_t tick;
	float ax_mps2;
	float ay_mps2;
	float gz_rps;
} IMU_DATA;

typedef struct
{
	I2C_CHANNEL channel;
	uint8_t i2cAddress;
} IMU_TASK_CONFIG;

///////////////////////////////////////////////////////////////////////////////
// MPU-9250 / MPU-6500 / MPU-9255 register map (gedeeld deel)

#define MPU_REG_SMPLRT_DIV        0x19
#define MPU_REG_CONFIG            0x1A
#define MPU_REG_GYRO_CONFIG       0x1B
#define MPU_REG_ACCEL_CONFIG      0x1C
#define MPU_REG_ACCEL_CONFIG2     0x1D
#define MPU_REG_INT_PIN_CFG       0x37
#define MPU_REG_ACCEL_XOUT_H      0x3B
#define MPU_REG_PWR_MGMT_1        0x6B
#define MPU_REG_WHO_AM_I          0x75

#define MPU_ADDR_LOW              0x68
#define MPU_ADDR_HIGH             0x69

// WHO_AM_I waardes die je kunt tegenkomen
#define MPU_WHOAMI_6500           0x70
#define MPU_WHOAMI_9250           0x71
#define MPU_WHOAMI_9255           0x73

// Configuratie gekozen voor nette resolutie en een bruikbare bandbreedte
// Accel: +-4g  -> 8192 LSB/g
// Gyro : +-1000 dps -> 32.8 LSB/(deg/s)

#define MPU_ACCEL_SENS_4G         8192.0f
#define MPU_GYRO_SENS_1000DPS     32.8f
#define GRAVITY_MPS2              9.80665f
#define DEG_TO_RAD                0.01745329252f

///////////////////////////////////////////////////////////////////////////////
// file globals

QueueHandle_t handle_IMUQueue = NULL;

static I2C_HANDLE *G_imuI2CHandle = NULL;
static uint8_t     G_imuAddress   = MPU_ADDR_LOW;

///////////////////////////////////////////////////////////////////////////////
// local function prototypes

static int16_t imu_CombineBytes(uint8_t msb, uint8_t lsb);
static bool imu_CheckWhoAmI(void);
static bool imu_Configure(void);

///////////////////////////////////////////////////////////////////////////////
// int16_t imu_CombineBytes(uint8_t msb, uint8_t lsb)

static int16_t imu_CombineBytes(uint8_t msb, uint8_t lsb)
{
	return (int16_t)((((int16_t)msb) << 8) | lsb);
}

///////////////////////////////////////////////////////////////////////////////
// bool imu_CheckWhoAmI(void)

static bool imu_CheckWhoAmI(void)
{
	uint8_t whoAmI = 0U;

	if (G_imuI2CHandle == NULL)
	{
		return false;
	}

	whoAmI = i2c_ReadSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_WHO_AM_I);

	if ((whoAmI == MPU_WHOAMI_6500) ||
	(whoAmI == MPU_WHOAMI_9250) ||
	(whoAmI == MPU_WHOAMI_9255))
	{
		return true;
	}

	return false;
}

///////////////////////////////////////////////////////////////////////////////
// bool imu_Configure(void)

static bool imu_Configure(void)
{
	if (G_imuI2CHandle == NULL)
	{
		return false;
	}

	// device reset
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_PWR_MGMT_1, 0x80);
	delay_ms(100);

	// clock source = auto select best available clock
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_PWR_MGMT_1, 0x01);
	delay_ms(10);

	// sample rate divider: 1 kHz / (1 + 0) = 1 kHz
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_SMPLRT_DIV, 0x00);

	// DLPF gyro ~92 Hz
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_CONFIG, 0x02);

	// gyro range +-1000 dps
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_GYRO_CONFIG, (0x02 << 3));

	// accel range +-4g
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_ACCEL_CONFIG, (0x01 << 3));

	// accel DLPF ~92 Hz
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_ACCEL_CONFIG2, 0x02);

	// bypass uit, standaard interrupt gedrag laten staan
	i2c_WriteSingleRegister(G_imuI2CHandle, G_imuAddress, MPU_REG_INT_PIN_CFG, 0x00);

	delay_ms(10);

	return true;
}

///////////////////////////////////////////////////////////////////////////////
// bool imu_Init(I2C_CHANNEL channel, uint8_t i2cAddress)

bool imu_Init(I2C_CHANNEL channel, uint8_t i2cAddress)
{
	bool initOk = true;

	// I2C library init
	if (i2c_Init() == false)
	{
		return false;
	}

	G_imuI2CHandle = i2c_GetChannelHandle(channel);
	G_imuAddress   = i2cAddress;

	if (G_imuI2CHandle == NULL)
	{
		return false;
	}

	// queue length = 1, zodat altijd alleen de nieuwste sample bewaard blijft
	if (handle_IMUQueue == NULL)
	{
		handle_IMUQueue = xQueueCreate(1, sizeof(IMU_DATA));
		if (handle_IMUQueue == NULL)
		{
			return false;
		}
	}

	initOk &= imu_CheckWhoAmI();
	initOk &= imu_Configure();

	return initOk;
}

///////////////////////////////////////////////////////////////////////////////
// void imu_Task(void *pvParameters)

void imu_Task(void *pvParameters)
{
	const IMU_TASK_CONFIG *config = (const IMU_TASK_CONFIG *)pvParameters;
	I2C_CHANNEL channel = I2C_CHANNEL_0;
	uint8_t i2cAddress = MPU_ADDR_LOW;

	static const TickType_t taskPeriodTicks = 1U;   // 1 ms
	TickType_t lastWakeTime = xTaskGetTickCount();

	uint8_t rawData[14];
	int16_t rawAx = 0;
	int16_t rawAy = 0;
	int16_t rawGz = 0;

	IMU_DATA imuData;

	// simpele offsets; later netjes calibreren
	static const float axBias_mps2 = 0.0f;
	static const float ayBias_mps2 = 0.0f;
	static const float gzBias_rps  = 0.0f;

	if (config != NULL)
	{
		channel = config->channel;
		i2cAddress = config->i2cAddress;
	}

	if (imu_Init(channel, i2cAddress) == false)
	{
		vPrintString("IMU init failed.\n");
		vTaskDelete(NULL);
	}

	if ((G_imuI2CHandle == NULL) || (handle_IMUQueue == NULL))
	{
		vPrintString("IMU task start failed.\n");
		vTaskDelete(NULL);
	}

	for (;;)
	{
		// Lees in 1 burst het standaard datablock vanaf ACCEL_XOUT_H
		i2c_ReadRegisters(G_imuI2CHandle, G_imuAddress, MPU_REG_ACCEL_XOUT_H, rawData, 14);

		rawAx = imu_CombineBytes(rawData[0],  rawData[1]);
		rawAy = imu_CombineBytes(rawData[2],  rawData[3]);
		rawGz = imu_CombineBytes(rawData[12], rawData[13]);

		imuData.tick    = xTaskGetTickCount();
		imuData.ax_mps2 = (((float)rawAx) / MPU_ACCEL_SENS_4G) * GRAVITY_MPS2 - axBias_mps2;
		imuData.ay_mps2 = (((float)rawAy) / MPU_ACCEL_SENS_4G) * GRAVITY_MPS2 - ayBias_mps2;
		imuData.gz_rps  = ((((float)rawGz) / MPU_GYRO_SENS_1000DPS) * DEG_TO_RAD) - gzBias_rps;

		// Alleen nieuwste sample bewaren
		xQueueOverwrite(handle_IMUQueue, &imuData);
		
		static uint32_t printCounter = 0;

		if (++printCounter >= 100)   // print every 500 ms
		{
			printCounter = 0;
			vPrintString("IMU tick=%lu ax=%ld mm/s2 ay=%ld mm/s2 gz=%ld mrad/s\r\n",(unsigned long)imuData.tick,(long)(imuData.ax_mps2 * 1000.0f),(long)(imuData.ay_mps2 * 1000.0f),(long)(imuData.gz_rps * 1000.0f) );
		}

		vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
	}
}
