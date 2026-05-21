
/*
 * Regelaar.h
 *
 * Created: 21/05/2026 20:01:35
 *  Author: raphv
 */ 
#ifndef REGELAAR_H_
#define REGELAAR_H_

///////////////////////////////////////////////////////////////////////////////
// functions declerations
void Regelaar_INIT(void);
float PIDregelaar(uint8_t motorIndex, float error);
float FeedForward(float alpha);

#endif /* REGELAAR_H_ */

