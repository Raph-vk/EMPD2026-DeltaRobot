/*
 * MotionEngine.h
 *
 * Motion planning and motor-servo entry points for the 1 ms control loop.
 */

#ifndef MOTIONENGINE_H_
#define MOTIONENGINE_H_

#include <stdbool.h>

#include "MachinePins.h" // for N_MOTORS

///////////////////////////////////////////////////////////////////////////////
// function prototypes
void SequenceRESET(void);
void BuildSequence(void);
bool RunSequence(void);

#endif /* MOTIONENGINE_H_ */
