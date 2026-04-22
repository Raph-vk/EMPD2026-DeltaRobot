/*
 * VisualisationTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#ifndef VISUALISATIONTASK_H_
#define VISUALISATIONTASK_H_

//////////////////////////////////////////////////////////////////////////////
// #define's
#define LAMP_GREEN     BIT_LAMP_GREEN
#define LAMP_ORANGE    BIT_LAMP_ORANGE
#define LAMP_RED       BIT_LAMP_RED

//////////////////////////////////////////////////////////////////////////////
// function prototypes

void port_AllLampsOff(void);
void port_SetLamp(uint8_t lampNumber);
void VisualisationTask(void *pvParameters);


#endif /* VISUALISATIONTASK_H_ */