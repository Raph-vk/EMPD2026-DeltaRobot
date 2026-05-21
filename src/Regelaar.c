/*
 * Regelaar.c
 *
 * Created: 21/05/2026 20:01:24
 *  Author: raphv
 */
#include "Regelaar.h"

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes

#include "vPrintString.h"

///////////////////////////////////////////////////////////////////////////////
#include "MachinePins.h" //for N_MOTORS

///////////////////////////////////////////////////////////////////////////////
static const float Tsample = 0.001f;

// Regelaarconstanten
static const float Kp    = 9.4821f;
static const float Tau_d = 0.0375f;
static const float Tau_f = 0.00125f;
static const float Tau_i = 0.1250f;


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
//void Regelaar_INIT(void)
/* 
 * Berekend (eenmalig) alle regelaar instellingen.
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
	for (uint8_t i = 0; i < N_MOTORS; i++)
	{
		motorHist[i].e_1 = 0.0f;
		motorHist[i].e_2 = 0.0f;

		motorHist[i].u_1 = 0.0f;
		motorHist[i].u_2 = 0.0f;
	}
}//end-function

//////////////////////////////////////////////////////////////////////////////
//float PIDregelaar(uint8_t motorIndex, float error)
/* 
 * input = motorindex en bepaalde fout
 *
 * Bereken van ouput spanning met digitale signaalbewerking door oude in/uitput waarden mee te nemen.
 *
 * returnwaarde = gewenste spanning 
 */
float PIDregelaar(uint8_t motorIndex, float error)
{
	 if (motorIndex >= N_MOTORS)
	 {
		 vPrintString("> Onbekende motor (Aoutput = 0V)");
		 return 0.0f;
	 }

	 MotorIOhistory_t *hist = &motorHist[motorIndex];

	//
	float u_0 = (Kp * (b0 * error + b1 * hist->e_1 + b2 * hist->e_2) - a1 * hist->u_1 - a2 * hist->u_2 ) / a0;

	 // Historie doorschuiven
	 hist->e_2 = hist->e_1;
	 hist->e_1 = error;

	 hist->u_2 = hist->u_1;
	 hist->u_1 = u_0;

	 return u_0;
}// end-function


//////////////////////////////////////////////////////////////////////////////
//float FeedForward(float alpha)
/* 
 * input =			gewenste hoekacceleratie (bovenarm)  
 * returnwaarde =	gewenste spanning 
 */
//////////////////////////////////
float FeedForward(float alpha)
{
	const float Je_totaal = 0.000020378; //2.0378e-05// kgm^2, totale traagheidsmoment op de motoras
	const float KaKt = 0.01016; // Nm/V (Ka [A/V] * Kt [Nm/A])
	return (Je_totaal * alpha) / KaKt;
}