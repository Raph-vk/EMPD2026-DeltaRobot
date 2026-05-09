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
#include "QuadratureCounters.h"

#include "vPrintString.h" 
#include "Map.h" // voor constrain() and fmap()

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board

#include "DAC4921Lib.h"
#include "QC7366Lib.h"

///////////////////////////////////////////////////////////////////////////////
// define's
#define GRIPPER BIT_GRIPPER

///////////////////////////////////////////////////////////////////////////////
// local type definitions SEQUENCE
typedef enum
{
    STEP_MOVE,
    STEP_HOLD,
    STEP_GRIPPER,
    STEP_END
} StepType;

typedef struct
{
    StepType type;

    float x;
    float y;
    float z;
    float Tmax;

    float Twait;
    Bool grab;
} SequenceStep;

#define MOVE(x_, y_, z_, t_)   { STEP_MOVE,    x_, y_, z_, t_, 0.0f, false }
#define HOLD(t_)               { STEP_HOLD,    0.0f, 0.0f, 0.0f, 0.0f, t_, false }
#define GRIP(g_, t_)           { STEP_GRIPPER, 0.0f, 0.0f, 0.0f, 0.0f, t_, g_ }
#define END_SEQ()              { STEP_END,     0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false }

///////////////////////////////////////////////////////////////////////////
// Function prototypes
Bool HoldPosition(const float holdArmPos_DegInput[N_MOTORS]);
void InitSequence(void);
Bool HoldCurrentPosition(float Twait);
Bool GripperAtCurrentPosition(const Bool Grab ,const float Twait);
Bool Move_ToSetpoint(float x_eindPos, float y_eindPos, float z_eindPos, float Tmax);
Bool RunSequence(void);


#endif /* MOTIONENGINE_H_ */
