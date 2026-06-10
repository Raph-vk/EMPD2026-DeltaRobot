/*
 * MotionProfiles.c
 *
 * Created: 03/06/2026 21:10:17
 *  Author: raphv
 */

#include "MotionProfiles.h"

static void thirdDegreeProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref);
static void fifthDegreeProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref);
static void sCurveProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref);


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//void motionProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
/*
 * Pakt correcte bewegingsprofiel om gemakkelijk te kunnen schakelen.
 * 
 * Invoer; afstand, maxTijd, geduurde tijd, referentiewaardes met pointerstruct
 */
void motionProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
{
	
	// Pas onderstaand bewegingsprofielfunctie
	fifthDegreeProfile(distance, Tm, elapsedTime_s, ref);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
//void thirdDegreeProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
/*
 * Symmetrisch derdegraads positieprofiel per segment.
 * Gebruikt lokale segmenttijd zodat positie, snelheid en acceleratie continu blijven.
 *
 * Invoer; afstand, maxTijd, geduurde tijd, referentiewaardes met pointerstruct
 */
static void thirdDegreeProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
{
	// Als ref leeg is
	if (ref == 0)
	{
		return;
	}
	
	//Alles eerst voor zekerheid op nul zetten.
	ref->pos = 0.0f;
	ref->vel = 0.0f;
	ref->acc = 0.0f;
	ref->klaar = false;

	// Als begintijd te klein is direct klaar
	if (Tm <= 0.0f)
	{
		ref->pos = distance;
		ref->klaar = true;
		return;
	}
	
	const float t1 = 0.25f * Tm;
	const float t2 = 0.75f * Tm;
	const float accMax = (8.0f * distance) / (Tm * Tm);
	const float accSlope = accMax / Tm;

	// 1: Als berekende elapsedTime_s nog voor de beweging ligt.
	if (elapsedTime_s <= 0.0f)
	{
		return;
	}
	// 2: Eerste kwart van de beweging: versnelling opbouwen
	else if (elapsedTime_s <= t1)
	{
		const float s = elapsedTime_s;

		ref->acc = 4.0f * accSlope *s;
		ref->vel = 2.0f * accSlope *s*s;
		ref->pos = (2.0f / 3.0f) * accSlope *s*s*s;
		return;
	}
	// 3; van versnellen naar vertragen
	else if (elapsedTime_s <= t2)
	{
		const float s = elapsedTime_s - t1;

		ref->acc = accMax - 4.0f * accSlope * s;
		
		ref->vel = accMax * s
				- 2.0f * accSlope *s*s
				+ 0.125f * accMax * Tm;

		ref->pos = 0.5f * accMax *s*s
				- (2.0f / 3.0f) * accSlope *s*s*s
				+ 0.125f * accMax * Tm * s
				+ (1.0f / 96.0f) * accMax * Tm*Tm;
		
		return;
	}
	// 4; van vertraging terug naar stilstand, actief remmen tot steeds minder remmen.
	else if (elapsedTime_s < Tm)
	{
		const float s = elapsedTime_s - t2;

		ref->acc = -accMax + 4.0f * accSlope * s;
		
		ref->vel = -accMax * s
				+ 2.0f * accSlope * s*s
				+ 0.125f * accMax * Tm;
		
		ref->pos = -0.5f * accMax * s*s
				+ (2.0f / 3.0f) * accSlope *s*s*s
				+ 0.125f * accMax * Tm * s
				+ (11.0f / 96.0f) * accMax * Tm*Tm;	
		return;
	}	
	// 5: Eindpunt vasthouden zodra de profiel-tijd verstreken is.
	else if (elapsedTime_s >= Tm)
	{
		ref->pos = distance;
		ref->klaar = true;
		return;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//void fifthDegreeProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
/*
 * Vijfdegraads positieprofiel.
 * Positie, snelheid en acceleratie zijn nul aan het begin/einde van de beweging.
 *
 * Invoer; afstand, maxTijd, geduurde tijd, referentiewaardes met pointerstruct
 */
static void fifthDegreeProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
{
	// Als ref leeg is
	if (ref == 0)
	{
		return;
	}

	// Alles eerst voor zekerheid op nul zetten.
	ref->pos = 0.0f;
	ref->vel = 0.0f;
	ref->acc = 0.0f;
	ref->klaar = false;

	// Als begintijd te klein is direct klaar
	if (Tm <= 0.0f)
	{
		ref->pos = distance;
		ref->klaar = true;
		return;
	}

	// 1: Als berekende elapsedTime_s nog voor de beweging ligt.
	if (elapsedTime_s <= 0.0f)
	{
		return;
	}
	// 2: Vijfdegraads profiel tussen start- en eindpunt.
	else if (elapsedTime_s < Tm)
	{
		const float u = elapsedTime_s / Tm;
		const float u2 = u * u;
		const float u3 = u2 * u;
		const float u4 = u3 * u;
		const float u5 = u4 * u;
		const float Tm2 = Tm * Tm;

		ref->pos = distance * (10.0f*u3 - 15.0f*u4 + 6.0f*u5);
		ref->vel = (distance / Tm) * (30.0f*u2 - 60.0f*u3 + 30.0f*u4);
		ref->acc = (distance / Tm2) * (60.0f*u - 180.0f*u2 + 120.0f*u3);
		return;
	}
	// 3: Eindpunt vasthouden zodra de profiel-tijd verstreken is.
	else if (elapsedTime_s >= Tm)
	{
		ref->pos = distance;
		ref->klaar = true;
		return;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//void sCurveProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
/*
 * Symmetrisch 7-stage S-curve profiel met rukbegrenzing.
 * De jerk wordt bepaald uit afstand en tijd, zodat het profiel exact eindigt op distance.
 *
 * Invoer; afstand, maxTijd, geduurde tijd, referentiewaardes met pointerstruct
 */
static void sCurveProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref)
{
	// Als ref leeg is
	if (ref == 0)
	{
		return;
	}

	// Alles eerst voor zekerheid op nul zetten.
	ref->pos = 0.0f;
	ref->vel = 0.0f;
	ref->acc = 0.0f;
	ref->klaar = false;

	// Als begintijd te klein is direct klaar
	if (Tm <= 0.0f)
	{
		ref->pos = distance;
		ref->klaar = true;
		return;
	}

	const float h = Tm / 7.0f;
	const float h2 = h * h;
	const float h3 = h2 * h;
	const float jerkMax = distance / (8.0f * h3);
	const float accMax = jerkMax * h;

	const float t1 = h;
	const float t2 = 2.0f * h;
	const float t3 = 3.0f * h;
	const float t4 = 4.0f * h;
	const float t5 = 5.0f * h;
	const float t6 = 6.0f * h;

	const float p1 = (1.0f / 6.0f) * jerkMax * h3;
	const float v1 = 0.5f * jerkMax * h2;
	const float a1 = accMax;

	const float p2 = (7.0f / 6.0f) * jerkMax * h3;
	const float v2 = 1.5f * jerkMax * h2;
	const float a2 = accMax;

	const float p3 = 3.0f * jerkMax * h3;
	const float v3 = 2.0f * jerkMax * h2;

	const float p4 = 5.0f * jerkMax * h3;
	const float v4 = 2.0f * jerkMax * h2;

	const float p5 = (41.0f / 6.0f) * jerkMax * h3;
	const float v5 = 1.5f * jerkMax * h2;
	const float a5 = -accMax;

	const float p6 = (47.0f / 6.0f) * jerkMax * h3;
	const float v6 = 0.5f * jerkMax * h2;
	const float a6 = -accMax;

	// 1: Als berekende elapsedTime_s nog voor de beweging ligt.
	if (elapsedTime_s <= 0.0f)
	{
		return;
	}
	// 2: Positieve jerk, versnelling opbouwen.
	else if (elapsedTime_s <= t1)
	{
		const float s = elapsedTime_s;
		const float s2 = s * s;
		const float s3 = s2 * s;

		ref->acc = jerkMax * s;
		ref->vel = 0.5f * jerkMax * s2;
		ref->pos = (1.0f / 6.0f) * jerkMax * s3;
		return;
	}
	// 3: Constante positieve versnelling.
	else if (elapsedTime_s <= t2)
	{
		const float s = elapsedTime_s - t1;
		const float s2 = s * s;

		ref->acc = a1;
		ref->vel = v1 + a1 * s;
		ref->pos = p1 + v1 * s + 0.5f * a1 * s2;
		return;
	}
	// 4: Negatieve jerk, versnelling terug naar nul.
	else if (elapsedTime_s <= t3)
	{
		const float s = elapsedTime_s - t2;
		const float s2 = s * s;
		const float s3 = s2 * s;

		ref->acc = a2 - jerkMax * s;
		ref->vel = v2 + a2 * s - 0.5f * jerkMax * s2;
		ref->pos = p2 + v2 * s + 0.5f * a2 * s2 - (1.0f / 6.0f) * jerkMax * s3;
		return;
	}
	// 5: Constante snelheid.
	else if (elapsedTime_s <= t4)
	{
		const float s = elapsedTime_s - t3;

		ref->acc = 0.0f;
		ref->vel = v3;
		ref->pos = p3 + v3 * s;
		return;
	}
	// 6: Negatieve jerk, vertraging opbouwen.
	else if (elapsedTime_s <= t5)
	{
		const float s = elapsedTime_s - t4;
		const float s2 = s * s;
		const float s3 = s2 * s;

		ref->acc = -jerkMax * s;
		ref->vel = v4 - 0.5f * jerkMax * s2;
		ref->pos = p4 + v4 * s - (1.0f / 6.0f) * jerkMax * s3;
		return;
	}
	// 7: Constante negatieve versnelling.
	else if (elapsedTime_s <= t6)
	{
		const float s = elapsedTime_s - t5;
		const float s2 = s * s;

		ref->acc = a5;
		ref->vel = v5 + a5 * s;
		ref->pos = p5 + v5 * s + 0.5f * a5 * s2;
		return;
	}
	// 8: Positieve jerk, vertraging terug naar nul.
	else if (elapsedTime_s < Tm)
	{
		const float s = elapsedTime_s - t6;
		const float s2 = s * s;
		const float s3 = s2 * s;

		ref->acc = a6 + jerkMax * s;
		ref->vel = v6 + a6 * s + 0.5f * jerkMax * s2;
		ref->pos = p6 + v6 * s + 0.5f * a6 * s2 + (1.0f / 6.0f) * jerkMax * s3;
		return;
	}
	// 9: Eindpunt vasthouden zodra de profiel-tijd verstreken is.
	else if (elapsedTime_s >= Tm)
	{
		ref->pos = distance;
		ref->klaar = true;
		return;
	}
}
