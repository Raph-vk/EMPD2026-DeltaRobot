/*
 * MotorControl.c
 *
 * Created: 10-04-2026
 * Author : Raph van Koeveringe
 *
 * Doel van deze module
 * ---------------------
 * Deze module bevat alleen motor-gerelateerde basisfuncties:
 * - motor-DAC-uitgangen aansturen;
 * - home-switches uitlezen;
 *
 * De homing-cyclus zelf staat in Homing.c. Daardoor blijft dit bestand een
 * overzichtelijke hardware/motor-laag.
 */

#include "MotorControl.h"

///////////////////////////////////////////////////////////////////////////////
// SYSTEM INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include <asf.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h> //for fabsf()

///////////////////////////////////////////////////////////////////////////////
// PROJECT / HAL INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include "vPrintString.h"

#include "PortIOLib.h"
#include "DAC4921Lib.h"
#include "QC7366Lib.h"       // LS7366R quadrature encoder counter

#include "Map.h"
#include "ControlTask.h"
#include "QuadratureCounters.h"

///////////////////////////////////////////////////////////////////////////////
/*
 * Homing-switches zijn in deze hardware actief-laag:
 * port_IsBitSet(...) == false betekent dat de switch actief is.
 */
static const uint8_t homeSwitchBit[N_MOTORS] =
{
    BIT_M1_HOME,
    BIT_M2_HOME,
    BIT_M3_HOME
};

///////////////////////////////////////////////////////////////////////////////
// publieke functies - motoruitgangen
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// void ZetMotorSpanning(uint8_t motorIndex, float spanningVolt)
/*
 * Stuurt een DAC-spanning naar een enkele motoruitgang.
 * Invoer: motorIndex kiest de motor, spanningVolt is de gewenste uitgangsspanning.
 * Uitvoer: geen returnwaarde; schrijft naar de bijbehorende DAC.
 */
uint32_t i = 0;
void ZetMotorSpanning(uint8_t motorIndex, float spanningVolt)
{
    if (motorIndex >= N_MOTORS)
    {
	    vPrintString("> ERROR: motorindex buiten bereik!\n");
	    return;
    }
	
	/*
	if (i > 100)
	{
		vPrintString("uDAC van %d is %.2f Volt.\n", motorIndex, spanningVolt);
		i = 0;
	}
	else
	{
		i++;
	}
	//spanningVolt = constrain(spanningVolt, -2.0f, 2.0f);
	*/
    dac_SetOutputVoltage(MotorDacChannel[motorIndex], spanningVolt);
}

///////////////////////////////////////////////////////////////////////////////
// void ZetMotorSpanningen(float spanningVolt)
/*
 * Stuurt dezelfde DAC-spanning naar alle motoruitgangen.
 * Invoer: spanningVolt is de gewenste uitgangsspanning voor iedere motor.
 * Uitvoer: geen returnwaarde; schrijft naar alle motor-DAC-kanalen.
 */
void ZetMotorSpanningen(float spanningVolt)
{	
    for (uint8_t motor = 0U; motor < N_MOTORS; motor++)
    {
        dac_SetOutputVoltage(MotorDacChannel[motor], spanningVolt);
    }
	
	//vPrintString("uDAC voor alle motoren is %.2f Volt.\n", spanningVolt);
}

///////////////////////////////////////////////////////////////////////////////
// publieke functies - home-switches
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// bool motor_IsHomeLimitActive(uint8_t motorIndex)
/*
 * Leest of de home-switch van een motor actief is.
 * Invoer: motorIndex kiest de motor waarvan de switch gelezen wordt.
 * Uitvoer: true wanneer de switch actief is of de index ongeldig is, anders false.
 */
bool motor_IsHomeLimitActive(uint8_t motorIndex)
{
    if (motorIndex >= N_MOTORS)
    {
        /*
         * Veilige default:
         * Bij een ongeldige index doen we alsof de switch actief is.
         */
        return true;
    }

    return !port_IsBitSet(homeSwitchBit[motorIndex]);
}

///////////////////////////////////////////////////////////////////////////////
// bool anyHomeSwitchActive(void)
/*
 * Controleert of minimaal een home-switch actief is.
 * Invoer: geen.
 * Uitvoer: true wanneer een van de motoren de home-switch raakt, anders false.
 */
bool anyHomeSwitchActive(void)
{
    for (uint8_t motor = 0U; motor < N_MOTORS; motor++)
    {
        if (motor_IsHomeLimitActive(motor))
        {
            return true;
        }
    }

    return false;
}