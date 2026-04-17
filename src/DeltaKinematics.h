/*
 * DeltaKinematics.h
 *
 * Forward and inverse kinematics for the delta robot.
 * The robot geometry is configured inside DeltaKinematics.c so the rest of the
 * project only has to call the FK/IK functions.
 *
 * The MotionEngine uses motor-shaft angles in radians. These APIs therefore
 * accept and return motor-radialen. If the geometry is solved in upper-arm
 * space internally, the conversion to/from motor space must happen inside
 * DeltaKinematics.c.
 *
 * Created: 10/04/2026
 * Author: Raph van Koeveringe
 */

#ifndef DELTAKINEMATICS_H_
#define DELTAKINEMATICS_H_

//////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <math.h>
#include <stdbool.h>
#include "MachinePins.h"

//////////////////////////////////////////////////////////////////////////////
// #define's
#define EPSILON_F			(1.0e-6f) // bijna nul
#define PI				(3.14159265358979323846f) // PI_floating

//////////////////////////////////////////////////////////////////////////////
// Functions
float PID_Controller(float error);
float FeedForward(float alphaRad);
bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorAnglesRad[3]);

#endif /* DELTAKINEMATICS_H_ */
