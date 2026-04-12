/*
 * MotionEngine.h
 *
 * Motion planning and joint servo entry points for the 1 ms control loop.
 */

#ifndef MOTIONENGINE_H_
#define MOTIONENGINE_H_

#include <stdbool.h>
#include <stdint.h>

#include "MotorControl.h"

typedef enum
{
	MOTION_MODE_DISABLED = 0,
	MOTION_MODE_HOLD,
	MOTION_MODE_MOVE_JOINTS,
	MOTION_MODE_MOVE_TCP,
	MOTION_MODE_WAIT,
	MOTION_MODE_GRIPPER
} MotionMode_t;

void MotionEngine_Init(void);
void MotionEngine_Disable(void);
void MotionEngine_RunTick(void);

bool MotionEngine_HasValidKinematics(void);

void MotionEngine_HoldPosition(float q1, float q2, float q3);
void MotionEngine_HoldCurrentPosition(void);

bool MotionEngine_StartMoveToTcp(float x, float y, float z, uint32_t moveTimeMs);
bool MotionEngine_StartMoveToJoints(float q1, float q2, float q3, uint32_t moveTimeMs);
bool MotionEngine_StartWait(uint32_t waitTimeMs);
void MotionEngine_SetGripperCommand(bool closed);

void MotionEngine_ResetSequence(void);
bool MotionEngine_RunSequence(void);

bool MotionEngine_IsBusy(void);
MotionMode_t MotionEngine_GetMode(void);

#endif /* MOTIONENGINE_H_ */
