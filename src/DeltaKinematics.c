/*
 * DeltaKinematics.c
 *
 * Forward and inverse kinematics for a classic 3-DOF delta robot.
 * Only this file knows the robot geometry, so geometry changes stay local.
 *
 * Created on: 10/04/2026
 * Author: Raph van Koeveringe
 */
#include "DeltaKinematics.h"
//////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <math.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////
// application includes
#include "MachinePins.h"
#include "vPrintString.h"

#define DEG_TO_RAD		(0.01745329252f)
#define RAD_TO_DEG		(57.2957795131f)
#define DELTAKINEMATICS_PRINT_IK	(0)

//////////////////////////////////////////////////////////////////////////////
//file globals
static float rBase = 200.0f;	// basisradius,schouderpunt [mm]
static float rPols = 40.0f;	// polsRadius [mm]				// platformradius [mm]
static float LengteBovenarm = 210.0f;	// bovenarm lengte [mm]
static float LengteOnderarm = 550.0f; // onderarm [mm]
static const float Bovenarm_thetaMax = 30.0 * DEG_TO_RAD; // bovenste limiet
static const float Bovenarm_thetaMin = -80.0 * DEG_TO_RAD; // onderste limiet

// constantes
//Mechanische hoeklimieten van de bovenarm.
static const float phi[N_MOTORS] = {0.0f, (120.0f * DEG_TO_RAD), (240.0f * DEG_TO_RAD)}; // 0deg, 120deg, 240deg in radialen voor rotatiematrix

//////////////////////////////////////////////////////////////////////////////
// bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorRad[N_MOTORS])
/*
 * Berekent de inverse kinematica voor de delta robot.
 * Invoer: tcpPosition_mm bevat de gewenste TCP-positie in millimeters.
 * Uitvoer: motorRad wordt gevuld met de benodigde motorhoeken in radialen.
 * Returnwaarde: true als alle motorhoeken geldig zijn, anders false.
 */
bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorRad[N_MOTORS]) // 
{
	#if DELTAKINEMATICS_PRINT_IK
		vPrintString("IK input was: X = %.1f, Y = %.1f, Z = %.1f [mm]\n", tcpPosition_mm[0], tcpPosition_mm[1], tcpPosition_mm[2]);
	#endif
	//////////////////////////////////////////////////////////////////////////
	// functie declaraties
	//x = tcpPosition_mm[0];
	//y = tcpPosition_mm[1];
	//z = tcpPosition_mm[2];
	
	// TCP-positie in het globale robotcoördinatenstelsel.
	//float tcpPosition_mm[N_MOTORS] = {x, y, z};
		
	//Per motor
	for (uint8_t motorIndex = 0; motorIndex < N_MOTORS; motorIndex++){
	
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

		//float theta_plus = atan2f(zE1_plus, xE1_plus - LengteBovenarm);
		float theta_plus = atan2f(zE1_plus, xE1_plus - rBase);

		// Als theta_plus binnen de mechanische limieten valt, direct gebruiken.
		if(theta_plus > Bovenarm_thetaMin && theta_plus < Bovenarm_thetaMax){

			// Geldige oplossing gevonden, opslaan en doorgaan naar volgende motor.
			motorRad[motorIndex] = -theta_plus * i_twk;
		}
		else
		{
			// Alleen als theta_plus ongeldig is, de min-oplossing berekenen.
			float zE1_minus = (-B - sqrt_d) / (2.0f * A);
			float xE1_minus = p * zE1_minus + q; 
			//float theta_minus = atan2f(zE1_minus, xE1_minus - LengteBovenarm);
			float theta_minus = atan2f(zE1_minus, xE1_minus - rBase);

			//Als geldige oplossing gevonden, opslaan en doorgaan naar volgende motor.
			if(theta_minus > Bovenarm_thetaMin && theta_minus < Bovenarm_thetaMax){		
				motorRad[motorIndex] = -theta_minus * i_twk;
			}
			// Beide oplossingen liggen buiten de mechanische limieten.
			else{
				return false;
			}
		}
	}//eind motor loop

	#if DELTAKINEMATICS_PRINT_IK
	vPrintString(
	"IK result: armDeg M1=%.3f, M2=%.3f, M3=%.3f graden\n",
	(motorRad[0] / i_twk) * RAD_TO_DEG,
	(motorRad[1] / i_twk) * RAD_TO_DEG,
	(motorRad[2] / i_twk) * RAD_TO_DEG );
	#endif


	// Alle motorhoeken zijn berekend en geldig
	return true;
}//eind IK-efficiente functie

//////////////////////////////////////////////////////////////////////////////
// bool DeltaKinematics_Forward(const float motorRad[N_MOTORS], float tcpPosition_mm[3])
/*
 * Berekent de forward kinematica voor de delta robot.
 * Invoer: motorRad bevat de motor-as hoeken in radialen.
 * Uitvoer: tcpPosition_mm wordt gevuld met de TCP-positie in millimeters.
 * Returnwaarde: true als de positie oplosbaar is, anders false.
 */
bool DeltaKinematics_Forward(const float motorRad[N_MOTORS], float tcpPosition_mm[3])
{
	// Stap 1: reserveer arrays voor de bolcentra van de drie onderarmen.
	// Elk bolcentrum is het elleboogpunt waar de onderarm aan de bovenarm vastzit.
	float cx[N_MOTORS];
	float cy[N_MOTORS];
	float cz[N_MOTORS];

	// Stap 2: bereken per motor het bolcentrum.
	// De motorhoek komt binnen als motor-as hoek en wordt eerst teruggerekend
	// naar de echte bovenarmhoek via de tandwielkastverhouding i_twk.
	for (uint8_t motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
	{
		float armTheta = -motorRad[motorIndex] / i_twk;
		float radius_xy = rBase - rPols + LengteBovenarm * cosf(armTheta);

		// Projecteer het bolcentrum naar het globale robotcoordinatenstelsel.
		// phi[motorIndex] bepaalt de 0, 120 of 240 graden orientatie van de arm.
		cx[motorIndex] = radius_xy * cosf(phi[motorIndex]);
		cy[motorIndex] = radius_xy * sinf(phi[motorIndex]);
		cz[motorIndex] = LengteBovenarm * sinf(armTheta);
	}

	// Stap 3: stel de eerste lineaire vergelijking op.
	// Deze ontstaat door bolvergelijking 1 van bolvergelijking 2 af te trekken.
	// Daardoor vallen de kwadratische x^2, y^2 en z^2 termen weg.
	float A1 = 2.0f * (cx[1] - cx[0]);
	float B1 = 2.0f * (cy[1] - cy[0]);
	float C1 = 2.0f * (cz[1] - cz[0]);
	float D1 = cx[1] * cx[1] - cx[0] * cx[0]
		+ cy[1] * cy[1] - cy[0] * cy[0]
		+ cz[1] * cz[1] - cz[0] * cz[0];

	// Stap 4: stel de tweede lineaire vergelijking op.
	// Deze ontstaat op dezelfde manier, maar nu uit bol 3 - bol 1.
	float A2 = 2.0f * (cx[2] - cx[0]);
	float B2 = 2.0f * (cy[2] - cy[0]);
	float C2 = 2.0f * (cz[2] - cz[0]);
	float D2 = cx[2] * cx[2] - cx[0] * cx[0]
		+ cy[2] * cy[2] - cy[0] * cy[0]
		+ cz[2] * cz[2] - cz[0] * cz[0];

	// Stap 5: controleer of de twee lineaire vergelijkingen oplosbaar zijn.
	// Een noemer dicht bij nul betekent dat x en y numeriek niet betrouwbaar
	// als functie van z kunnen worden bepaald.
	float denominator = A1 * B2 - A2 * B1;
	if (fabsf(denominator) < 1e-12f)
	{
		return false;
	}

	// Stap 6: schrijf x en y als lineaire functie van z.
	// Vorm: x = x_coeff * z + x_offset
	// Vorm: y = y_coeff * z + y_offset
	float x_coeff = (B1 * C2 - B2 * C1) / denominator;
	float x_offset = (B2 * D1 - B1 * D2) / denominator;

	float y_coeff = (A2 * C1 - A1 * C2) / denominator;
	float y_offset = (A1 * D2 - A2 * D1) / denominator;

	// Stap 7: vul x(z) en y(z) in in een van de bolvergelijkingen.
	// Hierdoor blijft er een kwadratische vergelijking over met alleen z.
	float quad_a = x_coeff * x_coeff + y_coeff * y_coeff + 1.0f;
	float quad_b = 2.0f * (x_coeff * (x_offset - cx[0])
		+ y_coeff * (y_offset - cy[0])
		- cz[0]);
	float quad_c = (x_offset - cx[0]) * (x_offset - cx[0])
		+ (y_offset - cy[0]) * (y_offset - cy[0])
		+ cz[0] * cz[0]
		- LengteOnderarm * LengteOnderarm;

	// Stap 8: controleer of de kwadratische vergelijking reele oplossingen heeft.
	// Een negatieve discriminant betekent dat de drie bollen elkaar niet snijden.
	float discriminant = quad_b * quad_b - 4.0f * quad_a * quad_c;
	if (discriminant < 0.0f)
	{
		return false;
	}

	// Stap 9: bereken beide mogelijke z-oplossingen.
	// De laagste z wordt gekozen, want bewegen altijd naar Z_negatief.
	float sqrt_d = sqrtf(discriminant);
	float z1 = (-quad_b + sqrt_d) / (2.0f * quad_a);
	float z2 = (-quad_b - sqrt_d) / (2.0f * quad_a);
	float z = z1 < z2 ? z1 : z2;

	// Stap 10: bereken de bijbehorende x en y en schrijf de TCP-positie terug.
	tcpPosition_mm[0] = x_coeff * z + x_offset;
	tcpPosition_mm[1] = y_coeff * z + y_offset;
	tcpPosition_mm[2] = z;

	return true;
}
