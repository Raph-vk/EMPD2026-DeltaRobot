
/*
 * IMUTask.h
 *
 * Created: 22/04/2026 08:37:48
 *  Author: raphv
 */ 
#ifndef IMU_TASK_H_
#define IMU_TASK_H_

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "I2CLib.h"

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
// globals

extern QueueHandle_t handle_IMUQueue;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

bool imu_Init(I2C_CHANNEL channel, uint8_t i2cAddress);
void imu_Task(void *pvParameters);

#endif /* IMU_TASK_H_ */
