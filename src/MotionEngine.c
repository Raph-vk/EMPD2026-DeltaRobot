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
#include "UserFrame.h"

///////////////////////////////////////////////////////////////////////////////
// sequence settings
#define MAX_SEQUENCE_STEPS		(360U)  //voor vaste memory grootte
///////////////////////////////////////////////////////////////////////////////
// globals vars
static uint16_t sequenceLength = 0;
static bool sequenceOverflow = false;

///////////////////////////////////////////////////////////////////////////////
// struct voor XY-positie
typedef struct
{
	float x;
	float y;
} Coordinate_t;

/* Middenplaat (bouten). */
const Coordinate_t M00 = {-80.00f,  80.00f};
const Coordinate_t M01 = {-40.00f,  80.00f};
const Coordinate_t M02 = {  0.00f,  80.00f};
const Coordinate_t M03 = { 40.00f,  80.00f};
const Coordinate_t M04 = { 80.00f,  80.00f};
const Coordinate_t M05 = {  0.00f,  50.00f};
const Coordinate_t M06 = {-43.30f,  25.00f};
const Coordinate_t M07 = { 43.30f,  25.00f};
const Coordinate_t M08 = {-43.30f, -25.00f};
const Coordinate_t M09 = { 43.30f, -25.00f};
const Coordinate_t M10 = {  0.00f, -50.00f};
const Coordinate_t M11 = {-80.00f, -80.00f};
const Coordinate_t M12 = {-40.00f, -80.00f};
const Coordinate_t M13 = {  0.00f, -80.00f};
const Coordinate_t M14 = { 40.00f, -80.00f};
const Coordinate_t M15 = { 80.00f, -80.00f};

/* Hoekplaat linksboven (moeren). */
const Coordinate_t LB00 = { -70.5f, 202.10f};
const Coordinate_t LB01 = {-110.21f, 190.90f};
const Coordinate_t LB02 = { -35.85f, 182.10f};
const Coordinate_t LB03 = {-139.78f, 162.10f};
const Coordinate_t LB04 = { -91.93f, 159.23f};
const Coordinate_t LB05 = {-139.78f, 122.10f};

/* Rechtsmidden (Moeren). */
const Coordinate_t RM00 = {175.63f,  60.0f};
const Coordinate_t RM01 = {210.27f,  40.0f};
const Coordinate_t RM02 = {183.86f,  0.00f};
const Coordinate_t RM03 = {220.43f,   0.00f};
const Coordinate_t RM04 = {175.63f, -60.0f};
const Coordinate_t RM05 = {210.27f, -40.0f};

/* Hoekplaat linksonder (moeren). */
const Coordinate_t LO00 = {-139.78f, -122.10f};
const Coordinate_t LO01 = {-139.78f, -162.10f};
const Coordinate_t LO02 = {-91.93f, -159.23f};
const Coordinate_t LO03 = {-110.21f, -190.90f};
const Coordinate_t LO04 = { -35.85f, -182.1f};
const Coordinate_t LO05 = { -70.5f, -202.1f};


///////////////////////////////////////////////////////////////////////////////
//wrappers om snel een sequentie te bouwen en aantepassen.
static inline void Hold(bool gripper, float time_s);
static inline void HoldXYZ(bool gripper, float time_s);
static inline void MoveJXYZ(float x_mm, float y_mm, float z_mm, float time_s);
static inline void MoveJDEG(float m1_deg, float m2_deg, float m3_deg, float time_s);
static inline void MoveLXYZ(float x_mm, float y_mm, float z_mm, float time_s);
static inline void MoveHopXYZ(float x_mm, float y_mm, float z_mm, float time_s);

///////////////////////////////////////////////////////////////////////////////
// static void BuildSequence(void)
/*
 * Bouwt de bewegingssequentie. Het vult alleen de staplijst.
 */
void BuildSequence(void)
{
	float waitGrip = 0.5f;
	float moveShort = 0.4f;
	float move = 0.5f;
	float moveLong = 1.30f;
	float moveHop = 1.60f;

	float zFly = -400.0f;
	float zPickBout = -456.0f;
	float zPickMoer = -460.0f;

	// Bouten minimaal 25 mm omhoog; hier 30 mm.
	float zBoutClear = zPickBout + 30.0f;
	float zMoerClear = zPickMoer + 5.0f;

	const Coordinate_t boutStart[6] = {M00, M01, M02, M03, M04, M05};
	const Coordinate_t boutDoel[6]  = {M15, M06, M10, M11, M13, M07};

	const Coordinate_t moerStart[6] = {RM00, RM01, RM02, RM03, RM04, RM05};
	const Coordinate_t moerDoel[6]  = {LB02, LO00, LB05, LO05, LB03, LB00};

	sequenceLength = 0;
	sequenceOverflow = false;

	MoveJXYZ(0.0f, 0.0f, zFly, moveLong);
	Hold(false, 0.5f);

	// Bouten plaatsen.
	for (uint8_t i = 0; i < 6; i++)
	{
		Coordinate_t pick = boutStart[i];
		Coordinate_t place = boutDoel[i];

		MoveHopXYZ(pick.x, pick.y, zBoutClear, moveHop);
		MoveLXYZ(pick.x, pick.y, zPickBout, moveShort);
		HoldXYZ(false, waitGrip);
		HoldXYZ(true, waitGrip);
		MoveLXYZ(pick.x, pick.y, zBoutClear, moveShort);

		MoveHopXYZ(place.x, place.y, zBoutClear, moveHop);
		MoveLXYZ(place.x, place.y, zPickBout, moveShort);
		HoldXYZ(false, waitGrip);
		MoveLXYZ(place.x, place.y, zBoutClear, moveShort);
	}

	MoveHopXYZ(0.0f, 0.0f, zFly, moveHop);

	// Moeren verplaatsen.
	for (uint8_t i = 0; i < 6; i++)
	{
		Coordinate_t pick = moerStart[i];
		Coordinate_t place = moerDoel[i];

		MoveLXYZ(pick.x, pick.y, zBoutClear, moveHop); 
		MoveLXYZ(pick.x, pick.y, zPickMoer, moveShort);
		HoldXYZ(false, waitGrip);
		HoldXYZ(true, waitGrip);
		MoveLXYZ(pick.x, pick.y, zMoerClear, moveShort);

		MoveHopXYZ(place.x, place.y, zMoerClear, moveHop);
		MoveLXYZ(place.x, place.y, zPickMoer, moveShort);
		HoldXYZ(false, waitGrip);
		MoveLXYZ(place.x, place.y, zBoutClear, moveShort);
	}
	
	// Moeren terugleggen.
	for (uint8_t i = 0; i < 6; i++)
	{
		Coordinate_t pick = moerDoel[i];
		Coordinate_t place = moerStart[i];

		MoveLXYZ(pick.x, pick.y, zBoutClear, moveHop);
		MoveLXYZ(pick.x, pick.y, zPickMoer, moveShort);
		HoldXYZ(false, waitGrip);
		HoldXYZ(true, waitGrip);
		MoveLXYZ(pick.x, pick.y, zMoerClear, moveShort);

		MoveHopXYZ(place.x, place.y, zMoerClear, moveHop);
		MoveLXYZ(place.x, place.y, zPickMoer, moveShort);
		HoldXYZ(false, waitGrip);
		MoveLXYZ(place.x, place.y, zBoutClear, moveShort);
	}

	MoveLXYZ(0.0f, 0.0f, zBoutClear, moveHop);

	// Bouten terugleggen.
	for (uint8_t i = 0; i < 6; i++)
	{
		Coordinate_t pick = boutDoel[i];
		Coordinate_t place = boutStart[i];

		MoveHopXYZ(pick.x, pick.y, zBoutClear, moveHop);
		MoveLXYZ(pick.x, pick.y, zPickBout, moveShort);
		HoldXYZ(false, waitGrip);
		HoldXYZ(true, waitGrip);
		MoveLXYZ(pick.x, pick.y, zBoutClear, moveShort);

		MoveHopXYZ(place.x, place.y, zBoutClear, moveHop);
		MoveLXYZ(place.x, place.y, zPickBout, moveShort);
		HoldXYZ(false, waitGrip);
		MoveLXYZ(place.x, place.y, zBoutClear, moveShort);
	}

	// Snelle demo-beweging.
	MoveLXYZ(0.0f, 0.0f, zBoutClear, move);
	Hold(false, waitGrip);
	MoveLXYZ(80.0f, 80.0f, (zBoutClear+20.0), moveShort);
	Hold(false, waitGrip);
	MoveLXYZ(80.0f, -80.0f, (zBoutClear+20.0), moveShort);
	Hold(false, waitGrip);
	MoveLXYZ(-80.0f, -80.0f, (zBoutClear+20.0), moveShort);
	Hold(false, waitGrip);
	MoveLXYZ(-80.0f, 80.0f, (zBoutClear+20.0), moveShort);
	Hold(false, waitGrip);
	MoveLXYZ(80.0f, 80.0f, (zBoutClear+20.0), moveShort);
	Hold(false, waitGrip);
	
	MoveLXYZ(0.0f, 0.0f, zBoutClear, moveShort);
	Hold(false, (waitGrip*4));
	
	MoveHopXYZ(125.0f, 0.0f, zBoutClear, moveHop);
	Hold(false, waitGrip);
	MoveHopXYZ(0.0f, 125.0f, zBoutClear, moveHop);
	Hold(false, waitGrip);
	MoveHopXYZ(0.0f, -125.0f, zBoutClear, moveHop);
	Hold(false, waitGrip);
	MoveLXYZ(0.0f, 0.0f, zBoutClear, moveShort);
	Hold(false, (waitGrip*4));
	


	MoveJXYZ(0.0f, 0.0f, zFly, moveLong);
	Hold(false, 0.8f);
}
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// bool RunSequence(void)
static uint16_t currentStep = 0;
typedef enum {STEP_HOLD_CURRENT,  STEP_HOLDXYZ_CURRENT, STEP_MOVEJ_XYZ,  STEP_MOVEJ_DEG,  STEP_MOVEL_XYZ, STEP_MOVEHOP_XYZ  } StepType_t;
typedef struct{ StepType_t type;   bool gripper;   float p1;   float p2;  float p3;  float time_s;  } SequenceStep_t;
typedef bool (*MoveXYZFunction_t)(float x_mm, float y_mm, float z_mm, float time_s);
static SequenceStep_t sequence[MAX_SEQUENCE_STEPS]; // De sequencielijst!

///////////////////////////////////////////////////////////////////////////////
// static bool RunUserFrameXYZStep(...)
static bool RunUserFrameXYZStep(const SequenceStep_t *step, MoveXYZFunction_t moveFunction)
{
	float baseXYZ_mm[3] = {0.0f, 0.0f, 0.0f};

	if (!UserFrame_ToBaseXY(step->p1, step->p2, step->p3, baseXYZ_mm))
	{
		return false;
	}

	return moveFunction(baseXYZ_mm[0], baseXYZ_mm[1], baseXYZ_mm[2], step->time_s);
}

///////////////////////////////////////////////////////////////////////////////
// static bool RunSequence(...)
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
		
		case STEP_HOLDXYZ_CURRENT:
		stepDone = HoldCurrentXYZPosition(step->gripper, step->time_s);
		break;

		case STEP_MOVEJ_XYZ:
		stepDone = RunUserFrameXYZStep(step, MoveJ_XYZt);
		break;

		case STEP_MOVEJ_DEG:
		stepDone = MoveJ_ArmDEG123t(step->p1, step->p2, step->p3, step->time_s);
		break;

		case STEP_MOVEL_XYZ:
		stepDone = RunUserFrameXYZStep(step, MoveL_XYZt);
		break;

		case STEP_MOVEHOP_XYZ:
		stepDone = RunUserFrameXYZStep(step, MoveHop_XYZt);
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

static inline void HoldXYZ(bool gripper, float time_s)
{
	StoreStep(STEP_HOLDXYZ_CURRENT, gripper, 0.0f, 0.0f, 0.0f, time_s);
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

static inline void MoveHopXYZ(float x_mm, float y_mm, float z_mm, float time_s)
{
	StoreStep(STEP_MOVEHOP_XYZ, false, x_mm, y_mm, z_mm, time_s);
}
