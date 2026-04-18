/*
 * MotionEngine.h
 *
 * Motion planning and motor-servo entry points for the 1 ms control loop.
 */

#ifndef MOTIONENGINE_H_
#define MOTIONENGINE_H_

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "DeltaKinematics.h"
#include "MachinePins.h"

#include "vPrintString.h" 
#include "Map.h" // voor constrain() and fmap()

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DAC4921Lib.h"
#include "QC7366Lib.h"
///////////////////////////////////////////////////////////////////////////
// Function prototypes

void MotionEngine_Init(void);
void MotionEngine_HoldCurrentPosition(void);
void MotionEngine_RunTick(void);
void MotionEngine_ResetSequence(void);
Bool MotionEngine_RunSequence(void);
void MotionEngine_Disable(void);

void ReadMotorPositions(float motorPos_Rad[N_MOTORS]);

void HoldPosition(float holdMotorPos_RadInput[N_MOTORS]);

Bool RunSequence(void);

Bool Move_ToSetpoint(float x_eindPos, float y_eindPos, float z_eindPos, float Tmax);

#endif /* MOTIONENGINE_H_ */
