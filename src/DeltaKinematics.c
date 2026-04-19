/*
 * DeltaKinematics.c
 *
 * Forward and inverse kinematics for a classic 3-DOF delta robot.
 * Only this file knows the robot geometry, so geometry changes stay local.
 *
 * Created on: 10/04/2026
 * Author: Raph van Koeveringe / Robbe
 */
//////////////////////////////////////////////////////////////////////////////
#include "DeltaKinematics.h"
//////////////////////////////////////////////////////////////////////////////
//Basic math values
#define DELTA_SIN120		(0.8660254037844386468f)
#define DELTA_COS120		(-0.5f)
#define DELTA_TAN60			(1.7320508075688772935f)
#define DELTA_SIN30			(0.5f)

//////////////////////////////////////////////////////////////////////////////
// system defines
#define Tsample			(0.001f)

//////////////////////////////////////////////////////////////////////////////
// typedefs
typedef struct
{
	float elleboogRadius_mm;
	float polsRadius_mm;
	float bovenarmLengte_mm;
	float onderarmLengte_mm;
} DeltaGeometry_t;

static uint8_t motorIndex = 0; // Motor index voor iteraties, 0,1,2 voor M1,M2,M3
///////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/* PID controller waarden 
 * error input -> returnwaarde = gewenste spanning 
 */
// Regelaarconstanten
static const float Kp = 0.0f;
static const float Ki = 0.0f;
static const float Kd = 0.0f;

// Interne geheugens
static float integral = 0.0f;
static float prevError = 0.0f;

float PID_Controller(float error)
{
	float derivative;
	float voltage;
	
	// I-actie
	integral += error * Tsample;

	// D-actie
	derivative = (error - prevError) / Tsample;

	// PID uitgang
	voltage = (Kp * error) + (Ki * integral) + (Kd * derivative);

	// Vorige error opslaan
	prevError = error;

	return voltage;
}

//////////////////////////////////////////////////////////////////////////////
/* FeedForward waarden bepalen 
 * gewenste hoekacceleratie (bovenarm) input -> returnwaarde = gewenste spanning 
 */
// constantes
static const float Je_totaal = 0.0f; // kgm^2, totale traagheidsmoment op de motoras
static const float KaKt = 0.0f;	// Nm/V (Ka [A/V] * Kt [Nm/A])

//intern geheugen
static float uFF = 0.0f;
//////////////////////////////////
float FeedForward(float alphaRad)
{
	//((Je1L+Je2L)*(alfag)/C2);
	uFF = (Je_totaal * alphaRad) / KaKt;

	return uFF;
}

//////////////////////////////////////////////////////////////////////////////////////
// @ROBBE vul de robotgeometrie waarden in, in millimeters.
static const DeltaGeometry_t RobotGeometry =
{
	200.0f,	// schouderRadius [mm]
	40.0f,	// polsRadius [mm]
	210.0f,	// bovenarm lengte [mm]
	550.0f	// onderarm lengte [mm]
};


//////////////////////////////////////////////////////////////////////////////
/* bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float jointAnglesRad[3])
 * Berekent de inverse kinematica van een delta robot.
 * Input: TCP positie in millimeters (X,Y,Z) in het robotcoördinatenstelsel.
 * Output: Motorhoeken in radialen (M1,M2,M3) in het robotcoördinatenstelsel.
 * Return: true als de berekening gelukt is, false als de TCP positie onbereikbaar is.
 */
// constantes

// intern geheugen
static float x= 0.0f;
static float y= 0.0f;
static float z= 0.0f;	

static const Bovenarm_thetaMaxDeg = 40;
static const Bovenarm_thetaMaxDeg = -80;

static float jointPos[N_MOTORS]; // bovenarmhoeken in radialen (M1,M2,M3)
static bool positionValid = false; // true als positie binnen bereik is, false als onbereikbaar

bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorRad[N_MOTORS]) // 
{
	//////////////////////////////////////////////////////////////////////////
	// functie declaraties
	x = tcpPosition_mm[0];
	y = tcpPosition_mm[1];
	z = tcpPosition_mm[2];
	
	float rBase = RobotGeometry[0];    // basisradius,schouderpunt [mm]
	float rPols = RobotGeometry[1];        // platformradius [mm]
	float LengteBovenarm = RobotGeometry[2];   // bovenarm [mm]
	float LengteOnderarm = RobotGeometry[3];   // onderarm [mm]

	//////////////////////////////////////////////////////////////////////////
	// @ROBBE/TESSA TODO: Berekeningen XYZ -> M1,M2,M3
	
	//////////////////////////////////////////////////////////////////////////
	// Returnen van de motorhoeken in radialen

    for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
    {
        motorRad[motorIndex] = (jointPos[motorIndex] * i_twk);
    }

	return positionValid;
}