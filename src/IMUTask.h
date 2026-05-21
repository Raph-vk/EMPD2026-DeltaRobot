
/*
 * IMUTask.h
 *
 * Created: 22/04/2026 08:37:48
 *  Author: raphv
 */ 
#ifndef IMU_TASK_H_
#define IMU_TASK_H_

///////////////////////////////////////////////////////////////////////////////
// globals

extern QueueHandle_t handle_IMUQueue;

///////////////////////////////////////////////////////////////////////////////
// function prototypes

bool imu_Init(I2C_CHANNEL channel, uint8_t i2cAddress);
void imu_Task(void *pvParameters);

#endif /* IMU_TASK_H_ */
