/*
 * UserFrame.c
 *
 * Created: 25/06/2026
 * Author: Robbe van Eekelen
 *
 * Vast user frame naar robotbase
 */

#include "UserFrame.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
///////////////////////////////////////////////////////////////////////////////
#include <stdbool.h>
#include <stddef.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// GEMETEN USER-FRAME WAARDEN
///////////////////////////////////////////////////////////////////////////////
#define USERFRAME_ORIGIN_X_MM	(0.0f)
#define USERFRAME_ORIGIN_Y_MM	(0.0f)

#define USERFRAME_COS_THETA		(1.0f)
#define USERFRAME_SIN_THETA		(0.0f)

/*
 * false = oude robotbasis-coordinaten gebruiken.
 * true  = X/Y uit de sequence omrekenen via het user frame.
 */
#define USERFRAME_ENABLED		(false)

/*
 * Pas op true zetten nadat de gemeten waarden hierboven zijn ingevuld.
 */
#define USERFRAME_VALID			(false)

///////////////////////////////////////////////////////////////////////////////
// bool UserFrame_IsValid(void)
bool UserFrame_IsValid(void)
{
	/* Als het user frame uit staat, mag de robot gewoon bewegen. */
	if (!USERFRAME_ENABLED)
	{
		return true;
	}

	return USERFRAME_VALID;
}

///////////////////////////////////////////////////////////////////////////////
// bool UserFrame_ToBaseXY(...)
bool UserFrame_ToBaseXY(float userX_mm,
						float userY_mm,
						float baseZ_mm,
						float baseXYZ_mm[3])
{
	if (baseXYZ_mm == NULL)
	{
		return false;
	}

	/* User frame uit: oude werking, direct robotbasis gebruiken. */
	if (!USERFRAME_ENABLED)
	{
		baseXYZ_mm[0] = userX_mm;
		baseXYZ_mm[1] = userY_mm;
		baseXYZ_mm[2] = baseZ_mm;

		return true;
	}

	/* User frame aan, maar nog geen geldige meetwaarden ingesteld. */
	if (!USERFRAME_VALID)
	{
		return false;
	}

	/* Alleen X en Y worden omgerekend. Z blijft absolute robotbasis-Z. */
	baseXYZ_mm[0] =
		USERFRAME_ORIGIN_X_MM +
		USERFRAME_COS_THETA * userX_mm -
		USERFRAME_SIN_THETA * userY_mm;

	baseXYZ_mm[1] =
		USERFRAME_ORIGIN_Y_MM +
		USERFRAME_SIN_THETA * userX_mm +
		USERFRAME_COS_THETA * userY_mm;

	baseXYZ_mm[2] = baseZ_mm;

	return true;
}