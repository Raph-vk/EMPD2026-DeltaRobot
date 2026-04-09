/*
 * ControlTask.h
 *
 * Created: 23-11-2023 14:40:03
 *  Author: rasmsmee
 */ 


#ifndef CONTROLTASK_H_
#define CONTROLTASK_H_

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void ClockInterruptHandler(uint32_t id, uint32_t mask);
void ControlLoop(void);
void ControlTask(void *pvParameters);

#endif /* CONTROLTASK_H_ */