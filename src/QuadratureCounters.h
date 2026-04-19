/*
 * QuadratureCounters.h
 *
 * Created: 23-11-2023 11:56:06
 *  Author: rasmsmee
 */ 


#ifndef QUADRATURECOUNTERS_H_
#define QUADRATURECOUNTERS_H_

///////////////////////////////////////////////////////////////////////////////
// include's
#include "MachinePins.h"
///////////////////////////////////////////////////////////////////////////////
// function prototypes

void QCEncodersSetup(void);
void QCEncodersShowCount(const char *idString);
void QCEncodersClearCount(void);

#endif /* QUADRATURECOUNTERS_H_ */