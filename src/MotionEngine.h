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
bool RunSequence(void);

bool HoldPosition(const float holdArmPos_RAD[N_MOTORS]);

bool HoldCurrentPosition(float waitTime_s);
bool GripperAtCurrentPosition(bool grab, float waitTime_s);
bool Move_ToSetpoint(float x_mm, float y_mm, float z_mm, float maxTime_s);

#endif /* MOTIONENGINE_H_ */
