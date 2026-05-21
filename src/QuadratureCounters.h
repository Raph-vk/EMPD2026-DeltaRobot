/*
 * QuadratureCounters.h
 *
 * Created: 23-11-2023 11:56:06
 *  Author: rasmsmee
 */ 


#ifndef QUADRATURECOUNTERS_H_
#define QUADRATURECOUNTERS_H_

#include "MachinePins.h" //for N_MOTORS
///////////////////////////////////////////////////////////////////////////////
// function prototypes

void QCEncodersSetup(void);
void QCEncodersShowCount(const char *idString);
void QCEncodersClearCount(void);

void ReadMotorPositions(float motorPos_Rad[N_MOTORS]);


#endif /* QUADRATURECOUNTERS_H_ */