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
// #includes
#include <math.h>
#include "DeltaKinematics.h"

//////////////////////////////////////////////////////////////////////////////
// #define's
#define EPSILON_F			(1.0e-6f) // bijna nul
#define PI_F				(3.14159265358979323846f) // pi_floating

//Basic math
#define DELTA_SIN120		(0.8660254037844386468f)
#define DELTA_COS120		(-0.5f)
#define DELTA_TAN60			(1.7320508075688772935f)
#define DELTA_SIN30			(0.5f)

#define T_sample			(0.001f)


typedef struct
{
	float baseRadius_mm;
	float endEffectorRadius_mm;
	float upperArmLength_mm;
	float forearmLength_mm;
} DeltaGeometry_t;

/*
 * @ROBBE vul de waarden hieronder in, in millimeters.
 *
 * baseRadius_mm en endEffectorRadius_mm zijn de afstanden van het robotcentrum
 * naar het aangrijppunt van de arm-kinematica in het model dat hieronder wordt
 * gebruikt. Dus niet de driehoekszijde, maar de radius waarmee de berekening werkt.
 */
static const DeltaGeometry_t RobotGeometry =
{
	0.0f,	// baseRadius
	0.0f,	// endEffectorRadius
	0.0f,	// bovenarm lengte
	0.0f	// onderarm lengte
};


//////////////////////////////////////////////////////////////////////////////
/* PID controller waarden 
 * error input -> returnwaarde = gewenste spanning 
 */
// Regelaarconstanten
static const float Kp = 1.0f;
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
	integral += error * T_sample;

	// D-actie
	derivative = (error - prevError) / T_sample;

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
// Regelaarconstanten
static const double Je_totaal = 0.0;
static const double KaKt = 0.0;
static double uFF = NULL;

float FeedForward(double alpha_Gewenst)
{
	//((Je1L+Je2L)*(alfag)/C2);
	uFF = (Je_totaal * alpha_Gewenst)/KaKt;

	return uFF;
}



bool DeltaKinematics_Inverse(const double tcpPosition_mm[3]) //XYZ
{
//////////////////////////////////////////////////////////////////////////
	// Functie declaraties
	double x_endPos = tcpPosition_mm[0]; //X
	double y_endPos = tcpPosition_mm[1]; //Y
	double z_endPos = tcpPosition_mm[2]; //Z
	
	double jointAnglesRad[3] = NULL;
	double theta_A0 = 0.0;
	double theta_A1 = 0.0;
	double theta_A2 = 0.0;
//////////////////////////////////////////////////////////////////////////

	
	jointAnglesRad[0] = theta_A0;
	jointAnglesRad[1] = theta_A1;
	jointAnglesRad[2] = theta_A2;

	return jointAnglesRad[3];
}



bool DeltaKinematics_Forward(const double jointAnglesRad[3]) //M1 M2 M3
{
//////////////////////////////////////////////////////////////////////////
	// functie declaraties
	double theta_A0 = jointAnglesRad[0];
	double theta_A1 = jointAnglesRad[1];
	double theta_A2 = jointAnglesRad[2];
	
	double tcpPosition[3] = NULL;
	double x = NULL;
	double y = NULL;
	double z = NULL;
//////////////////////////////////////////////////////////////////////////
	
	
	// Berekeningen
	
	
	tcpPosition[0] = x;
	tcpPosition[1] = y;
	tcpPosition[2] = z;

	return tcpPosition[3]
}