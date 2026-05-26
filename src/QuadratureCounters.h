/*
 * QuadratureCounters.h
 *
 * Created: 23-11-2023 11:56:06
 *  Author: rasmsmee
 * Aangepast: Raph van Koeveringe
 */ 


#ifndef QUADRATURECOUNTERS_H_
#define QUADRATURECOUNTERS_H_

#include "MachinePins.h" //for N_MOTORS
///////////////////////////////////////////////////////////////////////////////
// function prototypes

void QCEncodersSetup(void);
void QCEncodersShowCount(const char *idString);
void EncodersClearCount(void);
void EncoderClearCount(uint8_t qcChannel);

void LeesMotorPositiesRad(float motorPos_Rad[N_MOTORS]);
void LeesArmPositiesRad(float armPos_Rad[N_MOTORS]);
void LeesArmPositieRad(uint8_t motorIndex, float armPos_Rad[N_MOTORS]);

#endif /* QUADRATURECOUNTERS_H_ */