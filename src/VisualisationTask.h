/*
 * VisualisationTask.h
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
#ifndef VISUALISATIONTASK_H_
#define VISUALISATIONTASK_H_

//////////////////////////////////////////////////////////////////////////////
// function prototypes

void port_AllLampsOff(void);
void port_SetLamp(uint8_t lampNumber);
void VisualisationTask(void *pvParameters);


#endif /* VISUALISATIONTASK_H_ */
