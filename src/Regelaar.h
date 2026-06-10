
/*
 * Regelaar.h
 *
 * Created: 10/06/2026 19:48:05
 *  Author: raphv
 */ 
#ifndef REGELAAR_H_
#define REGELAAR_H_

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>

///////////////////////////////////////////////////////////////////////////////
// function prototypes
void Regelaar_INIT(void);
float PIDregelaar(uint8_t motorIndex, float error);
void removeRegelaarHistory(uint8_t arm);
float FeedForward(float alpha);


#endif /* REGELAAR_H_ */