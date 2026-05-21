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
#include <math.h>

//////////////////////////////////////////////////////////////////////////////
//Basic math values
#define DELTA_SIN120		(0.8660254037844386468f)
#define DELTA_COS120		(-0.5f)
#define DELTA_TAN60			(1.7320508075688772935f)
#define DELTA_SIN30			(0.5f)


//////////////////////////////////////////////////////////////////////////////
//file globals
static float rBase = 200.0f;	// basisradius,schouderpunt [mm]
static float rPols = 40.0f;	// polsRadius [mm]				// platformradius [mm]
static float LengteBovenarm = 210.0f;	// bovenarm lengte [mm]
static float LengteOnderarm = 550.0f; // onderarm [mm]
static const float Bovenarm_thetaMax = 0.6981317008f; // 40deg * (PI/180)
static const float Bovenarm_thetaMin = -1.3962634035f; // -80deg * (PI/180)
static uint8_t motorIndex = 0; // Motor index voor iteraties, 0,1,2 voor M1,M2,M3

//////////////////////////////////////////////////////////////////////////////
/* bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float jointAnglesRad[3])
 * Inverse kinematica: bereken per motor de benodigde bovenarmhoek
 * voor dezelfde TCP-positie. Door de TCP telkens naar het lokale
 * motorvlak te roteren kan dezelfde 2D-berekening voor M1, M2 en M3
 * gebruikt worden.
 */
// constantes
//Mechanische hoeklimieten van de bovenarm.

static const float phi[N_MOTORS] = {0.0f, 2.0943951024f, 4.1887902048f}; // 0deg, 120deg, 240deg in radialen voor rotatiematrix

Bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorRad[N_MOTORS]) // 
{
	//////////////////////////////////////////////////////////////////////////
	// functie declaraties
	//x = tcpPosition_mm[0];
	//y = tcpPosition_mm[1];
	//z = tcpPosition_mm[2];
	
	// TCP-positie in het globale robotcoördinatenstelsel.
	//float tcpPosition_mm[N_MOTORS] = {x, y, z};
		
	//Per motor
	for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++){
	
		// Rotatiematrix rond de z-as. Hiermee wordt de globale TCP-positie
		// omgerekend naar het lokale coordinatenstelsel van motor k.
		/*float Rz[3][3] = 
		{
			{ cosf(phi[motorIndex]),  sinf(phi[motorIndex]), 0.0f },
			{-sinf(phi[motorIndex]),  cosf(phi[motorIndex]), 0.0f },
			{ 0.0f,       			0.0f,  1.0f }
		};*/ // efficienter toegepast in onderstaande berekeningen
		
		// Lokale TCP-positie gezien vanuit motor.
		//Enkel essenciële berekening uitvoeren voor de rotatie, zonder volledige matrixvermenigvuldiging:
		float Pk[3];
		Pk[0] = cosf(phi[motorIndex]) * tcpPosition_mm[0] + sinf(phi[motorIndex]) * tcpPosition_mm[1];	// lokale x
		Pk[1] = -sinf(phi[motorIndex]) * tcpPosition_mm[0] + cosf(phi[motorIndex]) * tcpPosition_mm[1];	// lokale y
		Pk[2] = tcpPosition_mm[2];  // lokale z (geen verandering in z-coördinaat bij rotatie rond z-as)

		// Overzichtelijk en rekentechnisch effienter berekenen
		float x_pols = Pk[0] + rPols;


		// Resterende kwadratische lengte van de onderarm in het lokale x/z-vlak.
		// Negatief betekent dat de zijwaartse afstand groter is dan de onderarm.
		float L_oa_eff = LengteOnderarm * LengteOnderarm - Pk[1] * Pk[1];
		if(L_oa_eff < 0.0f){
			return false; //invalide positie, onderarm kan niet zover zijwaarts reiken
		}

		// Noemer van de lijnvergelijking tussen motorpunt en TCP. Bij bijna nul
		// ontstaat een singuliere of numeriek onbetrouwbare berekening.
		float dx_motor_target = 2 * (rBase  - x_pols);
		if(fabsf(dx_motor_target) < 1e-12f){
			return false; // invalide positie, TCP ligt te dicht bij de lijn van motor k
		}
		
		// p en q beschrijven de lijn waarop het elleboogpunt van de bovenarm
		// moet liggen, afgeleid uit de afstandsvergelijkingen van boven- en
		// onderarm.
		float p = Pk[2] / (rBase - x_pols);
		float q = (L_oa_eff - LengteBovenarm * LengteBovenarm - x_pols * x_pols - (Pk[2] * Pk[2]) + (rBase * rBase)) / dx_motor_target;
		
		// Kwadratische vergelijking voor de mogelijke z-posities van het
		// elleboogpunt. Snijpunten komen overeen met mogelijke armconfiguraties.
		float A = p * p + 1.0f;
		float B = 2.0f * p * (q - rBase);
		float C = (q - rBase) * (q - rBase) - LengteBovenarm * LengteBovenarm;
		
		// Geen reele oplossing betekent dat de gekozen TCP-positie voor deze
		// motor niet haalbaar is.
		float discriminant = (B * B) - ( 4.0f * A * C);
		if( discriminant < 0.0f){
			//invalide positie, elleboogpunt kan niet op de lijn liggen vanwege te grote afstand tot motorpunt
			return false; 
		}
		
		float sqrt_d = sqrtf(discriminant);

		// Eerst alleen de plus-oplossing berekenen.
		float zE1_plus = (-B + sqrt_d) / (2.0f * A);
		float xE1_plus = p * zE1_plus + q;

		float theta_plus = atan2f(zE1_plus, xE1_plus - LengteBovenarm);

		// Als theta_plus binnen de mechanische limieten valt, direct gebruiken.
		if(theta_plus > Bovenarm_thetaMin && theta_plus < Bovenarm_thetaMax){

			// Geldige oplossing gevonden, opslaan en doorgaan naar volgende motor.
			motorRad[motorIndex] = theta_plus * i_twk;
		}
		else{
			// Alleen als theta_plus ongeldig is, de min-oplossing berekenen.
			float zE1_minus = (-B - sqrt_d) / (2.0f * A);
			float xE1_minus = p * zE1_minus + q; 
			float theta_minus = atan2f(zE1_minus, xE1_minus - LengteBovenarm);

			//Als geldige oplossing gevonden, opslaan en doorgaan naar volgende motor.
			if(theta_minus > Bovenarm_thetaMin && theta_minus < Bovenarm_thetaMax){		
				motorRad[motorIndex] = theta_minus * i_twk;
			}
			// Beide oplossingen liggen buiten de mechanische limieten.
			else{
				return false;
			}
		}
	}//eind motor loop

	// Alle motorhoeken zijn berekend en geldig
	return true;
}//eind IK-efficiente functie