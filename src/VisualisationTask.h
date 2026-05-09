/*
 * VisualisationTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#ifndef VISUALISATIONTASK_H_
#define VISUALISATIONTASK_H_

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// library & HAL includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"

#include "I2CLib.h"  //voor scherm intergratie
#include "u8g2.h" //Voor scherm intergratie

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "ApplicationTasks.h"
#include "ControlTask.h"
#include "bits.h"


//////////////////////////////////////////////////////////////////////////////
// #includes
#include "LEDLib.h"

//////////////////////////////////////////////////////////////////////////////
// #define's
#define LAMP_GREEN     BIT_LAMP_GREEN
#define LAMP_ORANGE    BIT_LAMP_ORANGE
#define LAMP_RED       BIT_LAMP_RED

//////////////////////////////////////////////////////////////////////////////
// function prototypes

void port_AllLampsOff(void);
void port_SetLamp(uint8_t lampNumber);

Bool OLED_Init(void);
Bool OLED_DrawStatus(const char *line1, const char *line2, const char *line3, const char *line4, const char *line5);
void VisualisationTask(void *pvParameters);


#endif /* VISUALISATIONTASK_H_ */
