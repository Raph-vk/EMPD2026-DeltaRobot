/*
 * ControlTask.h
 *
 * Created: 23-11-2023 14:40:03
 *  Author: rasmsmee
 */ 


#ifndef CONTROLTASK_H_
#define CONTROLTASK_H_


///////////////////////////////////////////////////////////////////////////////
#define CLOCK_PIN PIN_30
#define NOODSTOP_PIN PIN_50
#define NOODSCHAKEL_PIN PIN_51
#define ESCONFOUT_PIN PIN_52


#define LAMP_GREEN     0
#define LAMP_ORANGE    1
#define LAMP_RED       2

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void ClockInterruptHandler(uint32_t id, uint32_t mask);
void ControlLoop(void);
void ControlTask(void *pvParameters);

#endif /* CONTROLTASK_H_ */