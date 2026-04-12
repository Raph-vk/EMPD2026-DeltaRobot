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

static bool GeometryIsValid(const DeltaGeometry_t *geometry)
{
	if (geometry == NULL)
	{
		return false;
	}

	return (geometry->baseRadius_mm > EPSILON_F)
		&& (geometry->endEffectorRadius_mm > EPSILON_F)
		&& (geometry->upperArmLength_mm > EPSILON_F)
		&& (geometry->forearmLength_mm > EPSILON_F);
}


static int DeltaKinematics_CalcAngleYZ(const DeltaGeometry_t *geometry, float x0, float y0, float z0, float *thetaRad)
{
	float y1 = 0.0f;
	float a = 0.0f;
	float b = 0.0f;
	float d = 0.0f;
	float yj = 0.0f;
	float zj = 0.0f;

	if ((thetaRad == NULL) || !GeometryIsValid(geometry) || (fabsf(z0) < EPSILON_F))
	{
		return -1;
	}

	y1 = -geometry->baseRadius_mm;
	y0 -= geometry->endEffectorRadius_mm;

	a = ((x0 * x0) + (y0 * y0) + (z0 * z0)
		+ (geometry->upperArmLength_mm * geometry->upperArmLength_mm)
		- (geometry->forearmLength_mm * geometry->forearmLength_mm)
		- (y1 * y1)) / (2.0f * z0);
	b = (y1 - y0) / z0;

	d = -((a + (b * y1)) * (a + (b * y1)))
		+ (geometry->upperArmLength_mm * ((b * b * geometry->upperArmLength_mm) + geometry->upperArmLength_mm));

	if (d < 0.0f)
	{
		return -1;
	}

	yj = (y1 - (a * b) - sqrtf(d)) / ((b * b) + 1.0f);
	zj = a + (b * yj);

	*thetaRad = atan2f(-zj, (y1 - yj));
	if (yj > y1)
	{
		*thetaRad += PI_F;
	}

	return 0;
}

bool DeltaKinematics_IsGeometryValid(void)
{
	return GeometryIsValid(&RobotGeometry);
}

bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float jointAnglesRad[3])
{
	float theta1 = 0.0f;
	float theta2 = 0.0f;
	float theta3 = 0.0f;

	if ((tcpPosition_mm == NULL) || (jointAnglesRad == NULL) || !GeometryIsValid(&RobotGeometry))
	{
		return false;
	}

	if (DeltaKinematics_CalcAngleYZ(&RobotGeometry, tcpPosition_mm[0], tcpPosition_mm[1], tcpPosition_mm[2], &theta1) != 0)
	{
		return false;
	}

	if (DeltaKinematics_CalcAngleYZ(
			&RobotGeometry,
			(tcpPosition_mm[0] * DELTA_COS120) + (tcpPosition_mm[1] * DELTA_SIN120),
			(tcpPosition_mm[1] * DELTA_COS120) - (tcpPosition_mm[0] * DELTA_SIN120),
			tcpPosition_mm[2],
			&theta2) != 0)
	{
		return false;
	}

	if (DeltaKinematics_CalcAngleYZ(
			&RobotGeometry,
			(tcpPosition_mm[0] * DELTA_COS120) - (tcpPosition_mm[1] * DELTA_SIN120),
			(tcpPosition_mm[1] * DELTA_COS120) + (tcpPosition_mm[0] * DELTA_SIN120),
			tcpPosition_mm[2],
			&theta3) != 0)
	{
		return false;
	}

	jointAnglesRad[0] = theta1;
	jointAnglesRad[1] = theta2;
	jointAnglesRad[2] = theta3;

	return true;
}

bool DeltaKinematics_Forward(const float jointAnglesRad[3], float tcpPosition_mm[3])
{
	float t = 0.0f;
	float y1 = 0.0f;
	float z1 = 0.0f;
	float x2 = 0.0f;
	float y2 = 0.0f;
	float z2 = 0.0f;
	float x3 = 0.0f;
	float y3 = 0.0f;
	float z3 = 0.0f;
	float dnm = 0.0f;
	float w1 = 0.0f;
	float w2 = 0.0f;
	float w3 = 0.0f;
	float a1 = 0.0f;
	float b1 = 0.0f;
	float a2 = 0.0f;
	float b2 = 0.0f;
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
	float discriminant = 0.0f;
	float theta1 = 0.0f;
	float theta2 = 0.0f;
	float theta3 = 0.0f;

	if ((jointAnglesRad == NULL) || (tcpPosition_mm == NULL) || !GeometryIsValid(&RobotGeometry))
	{
		return false;
	}

	theta1 = jointAnglesRad[0];
	theta2 = jointAnglesRad[1];
	theta3 = jointAnglesRad[2];

	t = RobotGeometry.baseRadius_mm - RobotGeometry.endEffectorRadius_mm;

	y1 = -(t + (RobotGeometry.upperArmLength_mm * cosf(theta1)));
	z1 = -(RobotGeometry.upperArmLength_mm * sinf(theta1));

	y2 = (t + (RobotGeometry.upperArmLength_mm * cosf(theta2))) * DELTA_SIN30;
	x2 = y2 * DELTA_TAN60;
	z2 = -(RobotGeometry.upperArmLength_mm * sinf(theta2));

	y3 = (t + (RobotGeometry.upperArmLength_mm * cosf(theta3))) * DELTA_SIN30;
	x3 = -y3 * DELTA_TAN60;
	z3 = -(RobotGeometry.upperArmLength_mm * sinf(theta3));

	dnm = ((y2 - y1) * x3) - ((y3 - y1) * x2);
	if (fabsf(dnm) < EPSILON_F)
	{
		return false;
	}

	w1 = (y1 * y1) + (z1 * z1);
	w2 = (x2 * x2) + (y2 * y2) + (z2 * z2);
	w3 = (x3 * x3) + (y3 * y3) + (z3 * z3);

	a1 = ((z2 - z1) * (y3 - y1)) - ((z3 - z1) * (y2 - y1));
	b1 = -(((w2 - w1) * (y3 - y1)) - ((w3 - w1) * (y2 - y1))) * 0.5f;

	a2 = -((z2 - z1) * x3) + ((z3 - z1) * x2);
	b2 = (((w2 - w1) * x3) - ((w3 - w1) * x2)) * 0.5f;

	a = (a1 * a1) + (a2 * a2) + (dnm * dnm);
	b = 2.0f * ((a1 * b1) + (a2 * (b2 - (y1 * dnm))) - (z1 * dnm * dnm));
	c = (b2 - (y1 * dnm)) * (b2 - (y1 * dnm))
		+ (b1 * b1)
		+ ((dnm * dnm) * ((z1 * z1) - (RobotGeometry.forearmLength_mm * RobotGeometry.forearmLength_mm)));

	discriminant = (b * b) - (4.0f * a * c);
	if (discriminant < 0.0f)
	{
		return false;
	}

	tcpPosition_mm[2] = -0.5f * (b + sqrtf(discriminant)) / a;
	tcpPosition_mm[0] = ((a1 * tcpPosition_mm[2]) + b1) / dnm;
	tcpPosition_mm[1] = ((a2 * tcpPosition_mm[2]) + b2) / dnm;

	return true;
}
