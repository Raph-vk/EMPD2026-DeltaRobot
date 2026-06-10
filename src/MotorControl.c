/*
 * MotorControl.c
 *
 * Created: 10-04-2026
 * Author : Raph van Koeveringe
 *
 * Doel van deze module
 * ---------------------
 * Deze module bevat alleen motor-gerelateerde basisfuncties:
 * - motor- en armposities uitlezen;
 * - motor-DAC-uitgangen aansturen;
 * - home-switches uitlezen;
 * - PID-regelspanning berekenen.
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
// CONSTANTEN
#define DEG_TO_RAD		(0.01745329251994329576923690768489f) //PI / 180.0f; // conversiefactor van graden naar radialen
#define RAD_TO_DEG		(57.295779513082320876798154814105f) // 180.0f / PI; // conversiefactor van radialen naar graden
#define maxErrorM		(10.0f * DEG_TO_RAD * i_twk) //

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
	
	vPrintString("uDAC voor alle motoren is %.2f Volt.\n", spanningVolt);
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

///////////////////////////////////////////////////////////////////////////////
// REGELAAR FUNCTIES 
///////////////////////////////////////////////////////////////////////////////
static const float Tsample = 0.001f;

// Regelaarconstanten
// Testopstelling
//static const float Kp    = 7.9101f;
//static const float Tau_d = 0.0375;
//static const float Tau_f = 0.00125;
//static const float Tau_i = 0.125;

// Deltarobot
static const float Kp    = 2.0257f;
static const float Tau_d = 0.056929;
static const float Tau_f = 0.0018976;
static const float Tau_i = 0.18976;




// Regelaarcoëfficiënten uit de Tustinmethode
static float a0;
static float a1;
static float a2;

static float b0;
static float b1;
static float b2;

//////////////////////////////////////////////////////////////////////////////
// struct om historie waarden error en outputvoltage op te slaan.
typedef struct
{
	// Fout-historie (input)
	float e_1;   // e[n-1]
	float e_2;   // e[n-2]

	// voltage-historie (output)
	float u_1;   // u[n-1]
	float u_2;   // u[n-2]

} MotorIOhistory_t;

static MotorIOhistory_t motorHist[N_MOTORS]; //aanmaken struct

///////////////////////////////////////////////////////////////////////////////
// void Regelaar_INIT(void)
/*
 * Berekent eenmalig de digitale PID-coefficienten en wist de historie.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; interne regelaarvariabelen worden geinitialiseerd.
 */
void Regelaar_INIT(void)
{
	//Eenmalig termen bepalen voor overzichtelijkheid
	const float Ti_term = (2.0f * Tau_i) / Tsample;
	const float Tf_term = (2.0f * Tau_f) / Tsample;
	const float Td_term = (2.0f * Tau_d) / Tsample;

	a0 = Ti_term * (1.0f + Tf_term); 	// u[n]
	a1 = -2.0f * Ti_term * Tf_term;		// u[n-1]
	a2 = -Ti_term * (1.0f - Tf_term);	// u[n-2]

	b0 = (1.0f + Td_term) * (1.0f + Ti_term);	// e[n]
	b1 = ((1.0f + Td_term) * (1.0f - Ti_term)) + ((1.0f - Td_term) * (1.0f + Ti_term));	// e[n-1]
	b2 = (1.0f - Td_term) * (1.0f - Ti_term); // e[n-2]

	//historie waarden op nul stellen
	for (uint8_t arm = 0; arm < N_MOTORS; arm++)
	{
		motorHist[arm].e_1 = 0.0f;
		motorHist[arm].e_2 = 0.0f;

		motorHist[arm].u_1 = 0.0f;
		motorHist[arm].u_2 = 0.0f;
	}
}//end-function

///////////////////////////////////////////////////////////////////////////////
// void removeRegelaarHistory(void)
/*
 * Wist de regelaar historie.
 * Invoer: arm/motor.
 * Uitvoer: geen returnwaarde; interne regelaarvariabelen worden geinitialiseerd.
 */
void removeRegelaarHistory(uint8_t arm)
{
	//fout nullen
	motorHist[arm].e_1 = 0.0f;
	motorHist[arm].e_2 = 0.0f;
	
	//spanningnullen
	motorHist[arm].u_1 = 0.0f;
	motorHist[arm].u_2 = 0.0f;
}//end-function

//////////////////////////////////////////////////////////////////////////////
// float PIDregelaar(uint8_t motorIndex, float error)
/*
 * Berekent de motorspanning met de digitale PID-regelaar.
 * Invoer: motorIndex kiest de motor, error is de regelfout in motor-radialen.
 * Uitvoer: gewenste motorspanning in volt.
 */
float PIDregelaar(uint8_t motorIndex, float error)
{
	 if (motorIndex >= N_MOTORS)
	 {
		 vPrintString("> Onbekende motor (Aoutput = 0V)");
		 return 0.0f;
	 }

	// bij een extreme positiefout naar error gaan, om optoeren te voorkomen.
	if ( fabsf(error) > maxErrorM)
	{
		vPrintString("> Te grote fout, namelijk: %.2f graden op de arm.\n", ( error * RAD_TO_DEG)/i_twk );
		ToState(STATE_FAULT);
		 return 0.0f;		
	}
	
	 MotorIOhistory_t *hist = &motorHist[motorIndex];

	//
	float u_0 = ( (Kp * (b0 * error + b1 * hist->e_1 + b2 * hist->e_2)) - (a1 * hist->u_1) - (a2 * hist->u_2) ) / a0;

	 // Historie doorschuiven
	 hist->e_2 = hist->e_1;
	 hist->e_1 = error;

	 hist->u_2 = hist->u_1;
	 hist->u_1 = u_0;

	 return u_0;
}// end-function

//////////////////////////////////////////////////////////////////////////////
// float FeedForward(float alpha)
/*
 * Berekent een feedforward-spanning uit de gewenste hoekacceleratie.
 * Invoer: alpha is de gewenste motorhoekacceleratie.
 * Uitvoer: feedforward-spanning in volt.
 */
float FeedForward(float alpha)
{
	const float Je_totaal = 0.000012557; // kgm^2, totale traagheidsmoment op de motoras
	const float KaKt = 0.01016; // Nm/V (Ka [A/V] * Kt [Nm/A])
	return (Je_totaal * alpha) / KaKt;
}