/*
 * PositionController.h
 *
 * Created: 18-9-2023 10:52:38
 *  Author: rasmsmee
 */ 


#ifndef POSITIONCONTROLLER_H_
#define POSITIONCONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

void Init_ControlParameters(void);
bool StartMoveTo(double q1, double q2, double q3, uint32_t moveTimeMs);
void HoldPosition(double q1, double q2, double q3);
bool MoveTo(void);

#endif /* POSITIONCONTROLLER_H_ */
