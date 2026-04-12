/*
 * DeltaKinematics.h
 *
 * Forward and inverse kinematics for the delta robot.
 * The robot geometry is configured inside DeltaKinematics.c so the rest of the
 * project only has to call the FK/IK functions.
 *
 * Created: 10/04/2026
 * Author: Raph van Koeveringe
 */

#ifndef DELTAKINEMATICS_H_
#define DELTAKINEMATICS_H_

//////////////////////////////////////////////////////////////////////////////
// #includes
#include <stdbool.h>

//////////////////////////////////////////////////////////////////////////////
// Functions
bool DeltaKinematics_IsGeometryValid(void);
bool DeltaKinematics_Forward(const float jointAnglesRad[3], float tcpPosition_mm[3]);
bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float jointAnglesRad[3]);

#endif /* DELTAKINEMATICS_H_ */
