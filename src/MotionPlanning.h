
/*
 * MotionPlanning.h
 *
 * Created: 03/06/2026 15:08:58
 *  Author: raphv
 */ 
#ifndef MOTIONPLANNING_H_
#define MOTIONPLANNING_H_

#include <stdbool.h>

#include "MachinePins.h" // for N_MOTORS

void MotionPlanning_RESET(void);

bool HoldCurrentPosition(bool grab, float waitTime_s);
bool MoveJ_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s);
bool MoveL_XYZt(float x_mm, float y_mm, float z_mm, float maxTime_s);
bool MoveJ_ArmDEG123t(float M1DEG, float M2DEG, float M3DEG, float maxTime_s);

#endif /* MOTIONPLANNING_H_ */
