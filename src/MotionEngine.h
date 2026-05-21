/*
 * MotionEngine.h
 *
 * Motion planning and motor-servo entry points for the 1 ms control loop.
 */

#ifndef MOTIONENGINE_H_
#define MOTIONENGINE_H_

#include "MachinePins.h" //voor N_MOTORS
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
    bool grab;
} SequenceStep;

#define MOVE(x_, y_, z_, t_)   { STEP_MOVE,    x_, y_, z_, t_, 0.0f, false }
#define HOLD(t_)               { STEP_HOLD,    0.0f, 0.0f, 0.0f, 0.0f, t_, false }
#define GRIP(g_, t_)           { STEP_GRIPPER, 0.0f, 0.0f, 0.0f, 0.0f, t_, g_ }
#define END_SEQ()              { STEP_END,     0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false }

///////////////////////////////////////////////////////////////////////////
// Function prototypes
bool HoldPosition(const float holdArmPos_DegInput[N_MOTORS]);
void InitSequence(void);
bool HoldCurrentPosition(float Twait);
bool GripperAtCurrentPosition(const bool Grab ,const float Twait);
bool Move_ToSetpoint(float x_eindPos, float y_eindPos, float z_eindPos, float Tmax);
bool RunSequence(void);


#endif /* MOTIONENGINE_H_ */
