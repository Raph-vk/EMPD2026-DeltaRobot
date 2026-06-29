/*
 * UserFrame.c
 *
 * Created: 25/06/2026
 * Author: Robbe van Eekelen
 *
 * Vast user frame naar robotbase.
 *
 * Rechterhandregel:
 * +X = vooruit
 * +Y = links
 * +Z = omhoog
 */

#include "UserFrame.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
///////////////////////////////////////////////////////////////////////////////
#include <stddef.h>

///////////////////////////////////////////////////////////////////////////////
// GEMETEN USER-FRAME WAARDEN
///////////////////////////////////////////////////////////////////////////////
/*
 * Dit zijn de opstartwaarden die gebruikt worden na reset/power-cycle.
 */
static float userFrameOriginX_mm = -2.987f;
static float userFrameOriginY_mm = 1.805f;

/*
 * Richting van userframe +X in de robotbasis.
 * cosTheta = X-component
 * sinTheta = Y-component
 */
static float userFrameCosTheta = 0.999951f;
static float userFrameSinTheta = -0.009913f;

/*
 * Rechterhandregel:
 * X vooruit, Y links, dus Z omhoog.
 *
 * Robotbasis +Z is eveneens omhoog.
 */
static float userFrameZSign = -1.0f;

///////////////////////////////////////////////////////////////////////////////
// bool UserFrame_ToBaseXY(...)
///////////////////////////////////////////////////////////////////////////////
bool UserFrame_ToBaseXY(float userX_mm,
                        float userY_mm,
                        float baseZ_mm,
                        float baseXYZ_mm[3])
{
    if (baseXYZ_mm == NULL)
    {
        return false;
    }

    /*
     * Rechterhandige rotatie:
     *
     * baseX = originX + cos(theta) * userX - sin(theta) * userY
     * baseY = originY + sin(theta) * userX + cos(theta) * userY
     *
     * +Y ligt dus links van +X, gezien vanaf +Z omhoog.
     */
    baseXYZ_mm[0] =
        userFrameOriginX_mm +
        userFrameCosTheta * userX_mm -
        userFrameZSign * userFrameSinTheta * userY_mm;

    baseXYZ_mm[1] =
        userFrameOriginY_mm +
        userFrameSinTheta * userX_mm +
        userFrameZSign * userFrameCosTheta * userY_mm;

    /*
     * Z wordt in de huidige sequences nog direct als robotbasis-Z gebruikt.
     */
    baseXYZ_mm[2] = baseZ_mm;

    return true;
}
