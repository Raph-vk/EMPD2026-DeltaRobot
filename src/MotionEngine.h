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

bool HoldCurrentPosition(bool grab, float waitTime_s);
bool MoveJ_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s);
bool MoveL_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s);
bool MoveJ_ArmDEG123t(float M1RAD, float M2RAD, float M3RAD, float maxTime_s);

#endif /* MOTIONENGINE_H_ */
