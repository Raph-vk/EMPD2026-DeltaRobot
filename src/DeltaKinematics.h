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

#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// function prototypes
bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorAnglesRad[3]);


#endif /* DELTAKINEMATICS_H_ */
