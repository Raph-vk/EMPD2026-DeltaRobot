/*
 * DisturbanceCompensation.h
 *
 * Created: 13-06-2026
 * Author: Robbe van Eekelen
 *
 * Meet X- en Y-verstoring van het robotframe en zet deze meting in een queue.
 */

#ifndef DISTURBANCECOMPENSATION_H_
#define DISTURBANCECOMPENSATION_H_

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "FreeRTOS.h"
#include "queue.h"

///////////////////////////////////////////////////////////////////////////////
// local type definitions
typedef struct
{
	float x_disturbance_mm;
	float y_disturbance_mm;
} DisturbanceMeasurement_t;

///////////////////////////////////////////////////////////////////////////////
// function prototypes
void DisturbanceCompensation_Init(QueueHandle_t queue);
void DisturbanceCompensation_UpdateQueue(void);

#endif /* DISTURBANCECOMPENSATION_H_ */