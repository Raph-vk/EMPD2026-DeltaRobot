/*
 *  MotionEngine.c
 *  
 *  MotionEngine for controlling the delta robot's motors to follow a desired trajectory.
 *
 *  Created: 10/04/2026
 *  Authors: Raph van Koeveringe
 */
#include "MotionEngine.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <stdint.h>
#include "MotionPlanning.h"

///////////////////////////////////////////////////////////////////////////////
// sequence settings
#define MAX_SEQUENCE_STEPS		(64U)  //voor vaste memory grootte
///////////////////////////////////////////////////////////////////////////////
// globals vars
static uint16_t sequenceLength = 0;
static bool sequenceOverflow = false;

///////////////////////////////////////////////////////////////////////////////
//wrappers om snel een sequentie te bouwen en aantepassen.
static inline void Hold(bool gripper, float time_s);
static inline void MoveJXYZ(float x_mm, float y_mm, float z_mm, float time_s);
static inline void MoveJDEG(float m1_deg, float m2_deg, float m3_deg, float time_s);
static inline void MoveLXYZ(float x_mm, float y_mm, float z_mm, float time_s);

///////////////////////////////////////////////////////////////////////////////
// static void BuildSequence(void)
/*
 * Bouwt de bewegingssequentie. Het vult alleen de staplijst.
 */
void BuildSequence(void)
{
	sequenceLength = 0;
	sequenceOverflow = false;
	MoveJDEG(0.0f, 0.0f, 0.0f, 1.0f);
	Hold(true, 0.1f);
 

	MoveLXYZ(80.0f, 80.0f, -480.0f, 0.250);
	Hold(true, 0.250f);
	MoveLXYZ(80.0f, -80.0f, -480.0f, 0.250);
	Hold(true, 0.250f);
	MoveLXYZ(-80.0f, -80.0f, -480.0f, 0.250);
	Hold(true, 0.250f);
	MoveLXYZ(-80.0f, 80.0f, -480.0f, 0.250);
	Hold(true, 0.250f);

	MoveJDEG(0.0f, 0.0f, 0.0f, 1.0f);
	Hold(false, 0.1f);
}
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// bool RunSequence(void)
static uint16_t currentStep = 0;
typedef enum {STEP_HOLD_CURRENT,  STEP_MOVEJ_XYZ,  STEP_MOVEJ_DEG,  STEP_MOVEL_XYZ  } StepType_t;
typedef struct{ StepType_t type;   bool gripper;   float p1;   float p2;  float p3;  float time_s;  } SequenceStep_t;
static SequenceStep_t sequence[MAX_SEQUENCE_STEPS]; // De sequencielijst!
/*
 * Voert de ingestelde bewegingssequentie stap voor stap uit.
 * Invoer: GEEN, wordt eenmaal per 1 ms clocktick aangeroepen.
 * Uitvoer: true wanneer de volledige sequentie klaar is, anders false.
 */
bool RunSequence(void)
{
	//Controle of sequentie niet voorbij is.
	if (sequenceOverflow || currentStep >= sequenceLength)
	{
		currentStep = 0;
		return true;
	}

	//Controle voor bij welke stap we nu zijn
	SequenceStep_t *step = &sequence[currentStep];
	bool stepDone = false;

	switch (step->type)
	{
		case STEP_HOLD_CURRENT:
		stepDone = HoldCurrentPosition(step->gripper, step->time_s);
		break;

		case STEP_MOVEJ_XYZ:
		stepDone = MoveJ_XYZt(step->p1, step->p2, step->p3, step->time_s);
		break;

		case STEP_MOVEJ_DEG:
		stepDone = MoveJ_ArmDEG123t(step->p1, step->p2, step->p3, step->time_s);
		break;

		case STEP_MOVEL_XYZ:
		stepDone = MoveL_XYZt(step->p1, step->p2, step->p3, step->time_s);
		break;

		default:
		stepDone = true;
		break;
	}

	if (stepDone)
	{
		currentStep++;
	}

	return false;
}

///////////////////////////////////////////////////////////////////////////////
// void SequenceRESET(void)
/*
 * Zet de bewegingssequentie terug naar de eerste stap.
 *
 * Uitvoer: geen returnwaarde; interne sequentie- en setupvlaggen worden gereset.
 */
void SequenceRESET(void)
{
	currentStep = 0;
	MotionPlanning_RESET();
}


///////////////////////////////////////////////////////////////////////////////
// static void StoreStep(...)
/*
 * Slaat een stap op in de sequentielijst. De motion wordt hier nog niet gestart;
 * RunSequence voert later steeds alleen de actieve stap uit.
 */
static void StoreStep(StepType_t type, bool gripper, float p1, float p2, float p3, float time_s)
{
	if (sequenceLength >= MAX_SEQUENCE_STEPS)
	{
		sequenceOverflow = true;
		return;
	}

	sequence[sequenceLength].type = type;
	sequence[sequenceLength].gripper = gripper;
	sequence[sequenceLength].p1 = p1;
	sequence[sequenceLength].p2 = p2;
	sequence[sequenceLength].p3 = p3;
	sequence[sequenceLength].time_s = time_s;
	sequenceLength++;
}

///////////////////////////////////////////////////////////////////////////////
// sequence step helpers
static inline void Hold(bool gripper, float time_s)
{
	StoreStep(STEP_HOLD_CURRENT, gripper, 0.0f, 0.0f, 0.0f, time_s);
}

static inline void MoveJXYZ(float x_mm, float y_mm, float z_mm, float time_s)
{
	StoreStep(STEP_MOVEJ_XYZ, false, x_mm, y_mm, z_mm, time_s);
}

static inline void MoveJDEG(float m1_deg, float m2_deg, float m3_deg, float time_s)
{
	StoreStep(STEP_MOVEJ_DEG, false, m1_deg, m2_deg, m3_deg, time_s);
}

static inline void MoveLXYZ(float x_mm, float y_mm, float z_mm, float time_s)
{
	StoreStep(STEP_MOVEL_XYZ, false, x_mm, y_mm, z_mm, time_s);
}