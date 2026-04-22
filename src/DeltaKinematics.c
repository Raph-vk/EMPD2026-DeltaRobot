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
// system defines
#define Tsample			(0.001f)


static float rBase = 200.0f;	// basisradius,schouderpunt [mm]
static float rPols = 40.0f;	// polsRadius [mm]				// platformradius [mm]
static float LengteBovenarm = 210.0f;	// bovenarm lengte [mm]
static float LengteOnderarm = 550.0f; // onderarm [mm]
static uint8_t motorIndex = 0; // Motor index voor iteraties, 0,1,2 voor M1,M2,M3

//////////////////////////////////////////////////////////////////////////////
/* PID controller waarden 
 * error input -> returnwaarde = gewenste spanning 
 */
// Regelaarconstanten
static const float Kp = 0.0f;
static const float Ki = 0.0f;
static const float Kd = 0.0f;
static float integral = 0.0f;
static float prevError = 0.0f;

float PID_Controller(float error)
{
	float derivative;
	float voltage;
	
	// I-actie
	integral += error * Tsample;
	// D-actie
	derivative = (error - prevError) / Tsample;
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
// constantes
static const float Je_totaal = 0.0f; // kgm^2, totale traagheidsmoment op de motoras
static const float KaKt = 0.0f;	// Nm/V (Ka [A/V] * Kt [Nm/A])

//intern geheugen
//////////////////////////////////
float FeedForward(float alphaRad)
{
	//((Je1L+Je2L)*(alfag)/C2);
	return (Je_totaal * alphaRad) / KaKt;
}


//////////////////////////////////////////////////////////////////////////////
/* bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float jointAnglesRad[3])
 * Berekent de inverse kinematica van een delta robot.
 * Input: TCP positie in millimeters (X,Y,Z) in het robotcoördinatenstelsel.
 * Output: Motorhoeken in radialen (M1,M2,M3) in het robotcoördinatenstelsel.
 * Return: true als de berekening gelukt is, false als de TCP positie onbereikbaar is.
 */
// constantes
// intern geheugen
/*
static float x= 0.0f;
static float y= 0.0f;
static float z= 0.0f;	

static const float Bovenarm_thetaMaxDeg = 40.0f;
static const float Bovenarm_thetaMinDeg = -80.0f;

static float jointPosRad[N_MOTORS]; // bovenarmhoeken in radialen (M1,M2,M3)
static Bool positionValid = false; // true als positie binnen bereik is, false als onbereikbaar

Bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorRad[N_MOTORS]) // 
{
	//////////////////////////////////////////////////////////////////////////
	// functie declaraties
	x = tcpPosition_mm[0];
	y = tcpPosition_mm[1];
	z = tcpPosition_mm[2];

	//float rBase = RobotGeometry.schouderRadius_mm;			// basisradius,schouderpunt [mm]
	//float rPols = RobotGeometry.polsRadius_mm;				// platformradius [mm]
	//float LengteBovenarm = RobotGeometry.bovenarmLengte_mm; // bovenarm [mm]
	//float LengteOnderarm = RobotGeometry.onderarmLengte_mm; // onderarm [mm]

	//////////////////////////////////////////////////////////////////////////
	// Inverse kinematica: bereken per motor de benodigde bovenarmhoek
	// voor dezelfde TCP-positie. Door de TCP telkens naar het lokale
	// motorvlak te roteren kan dezelfde 2D-berekening voor M1, M2 en M3
	// gebruikt worden.
	float phi_all[N_MOTORS] = {deg2rad(0.0f), deg2rad(120.0f), deg2rad(240.0f)};
	float theta_all[N_MOTORS] = {0.0f, 0.0f, 0.0f};

	// Mechanische hoeklimieten van de bovenarm omzetten naar radialen.
	float thetaMinRad = deg2rad(Bovenarm_thetaMinDeg);
	float thetaMaxRad = deg2rad(Bovenarm_thetaMaxDeg);

	// TCP-positie in het globale robotcoordinatenstelsel.
	float P0[N_MOTORS] = {x, y, z};
		
	for (int k = 0; k < N_MOTORS; k++){
		float phi = phi_all[k];
		
		// Rotatiematrix rond de z-as. Hiermee wordt de globale TCP-positie
		// omgerekend naar het lokale coordinatenstelsel van motor k.
		float Rz[3][3] = {
			{ cosf(phi),  sinf(phi), 0.0f },
			{-sinf(phi),  cosf(phi), 0.0f },
			{ 0.0f,       0.0f,      1.0f }
		};
		
		// Lokale TCP-positie gezien vanuit motor k.
		float Pk[3];
		Pk[0] = Rz[0][0] * P0[0] + Rz[0][1] * P0[1] + Rz[0][2] * P0[2];
		Pk[1] = Rz[1][0] * P0[0] + Rz[1][1] * P0[1] + Rz[1][2] * P0[2];
		Pk[2] = Rz[2][0] * P0[0] + Rz[2][1] * P0[1] + Rz[2][2] * P0[2];

		// In het lokale vlak wordt x/z gebruikt voor de armgeometrie.
		// y_loc is de zijwaartse afstand die al door de onderarm overbrugd
		// moet worden.
		float x_loc = Pk[0];
		float y_loc = Pk[1];
		float z_loc = Pk[2];

		// Resterende kwadratische lengte van de onderarm in het lokale x/z-vlak.
		// Negatief betekent dat de zijwaartse afstand groter is dan de onderarm.
		float L_oa_eff = LengteOnderarm * LengteOnderarm - y_loc * y_loc;
		if(L_oa_eff < 0.0f){
			return false;
		}

		// Noemer van de lijnvergelijking tussen motorpunt en TCP. Bij bijna nul
		// ontstaat een singuliere of numeriek onbetrouwbare berekening.
		float dx_motor_target = 2 * (rBase  - (x_loc + rPols));
		if(fabsf(dx_motor_target) < 1e-12f){
			return false;
		}
		
		// p en q beschrijven de lijn waarop het elleboogpunt van de bovenarm
		// moet liggen, afgeleid uit de afstandsvergelijkingen van boven- en
		// onderarm.
		float p = z_loc / (rBase - (x_loc + rPols));
		float q = (L_oa_eff - LengteBovenarm * LengteBovenarm - (x_loc + rPols) * (x_loc + rPols) - (z_loc * z_loc) + (rBase * rBase)) / dx_motor_target;
		
		// Kwadratische vergelijking voor de mogelijke z-posities van het
		// elleboogpunt. Snijpunten komen overeen met mogelijke armconfiguraties.
		float A = p * p + 1.0f;
		float B = 2.0f * p * (q - rBase);
		float C = (q - rBase) * (q - rBase) - LengteBovenarm * LengteBovenarm;
		
		// Geen reele oplossing betekent dat de gekozen TCP-positie voor deze
		// motor niet haalbaar is.
		float discriminant = (B * B) - ( 4.0f * A * C);
		if( discriminant < 0.0f){
			return false;
		}
		
		float sqrt_d = sqrtf(discriminant);

		// De kwadratische vergelijking levert maximaal twee mogelijke
		// elleboogposities op.
		float zE1_plus  = (-B + sqrt_d) / (2.0f * A);
		float zE1_minus = (-B - sqrt_d) / (2.0f * A);

		float xE1_plus  = p * zE1_plus  + q;
		float xE1_minus = p * zE1_minus + q;
		
		// Zet beide mogelijke elleboogposities om naar een bovenarmhoek.
		float theta_plus  = atan2f(zE1_plus,  xE1_plus  - LengteBovenarm);
		float theta_minus = atan2f(zE1_minus, xE1_minus - LengteBovenarm);
		
		// Alleen oplossingen binnen de mechanische hoeklimieten zijn bruikbaar.
		uint8_t geldig0 = (theta_plus  > thetaMinRad && theta_plus  < thetaMaxRad);
		uint8_t geldig1 = (theta_minus > thetaMinRad && theta_minus < thetaMaxRad);

		if (!geldig0 && !geldig1) {
			return 0;
		}
		
		// Als beide oplossingen geldig zijn, kies de hoek die het dichtst bij
		// nul ligt. Dit voorkomt een onnodig extreme armstand.
		float beste_thetaBovenArm = 0.0f;
		uint8_t eerste = 1;
		
		if(geldig0){
			beste_thetaBovenArm = theta_plus;
			eerste = 0;
		}
		if(geldig1){
			if(eerste || fabsf(theta_minus) < fabsf(beste_thetaBovenArm)){
				beste_thetaBovenArm = theta_minus;
			}
		}

		// Sla de gekozen bovenarmhoek voor motor k op.
		theta_all[k] = beste_thetaBovenArm;
		jointPosRad[k] = theta_all[k];
	}
			
	
	//////////////////////////////////////////////////////////////////////////
	// Controleren of hoekposities valide zijn binnen bereik
	positionValid = true;
    for (uint8_t motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
    {
	    if (jointPosRad[motorIndex] > thetaMaxRad && jointPosRad[motorIndex] < thetaMinRad) 
		{
			positionValid = false;
		}
		
    }

	//////////////////////////////////////////////////////////////////////////
	// Returnen van de motorhoeken in radialen
    for (motorIndex = 0; motorIndex < N_MOTORS; motorIndex++)
    {
        motorRad[motorIndex] = (jointPosRad[motorIndex] * i_twk);
    }

	return positionValid;
}
*/

//////////////////////////////////////////////////////////////////////////////
/* bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float jointAnglesRad[3])
 * EFFIECENTER GEMAAKT.
 */
// constantes
//Mechanische hoeklimieten van de bovenarm.
static const float Bovenarm_thetaMax = 0.6981317008f; // 40deg * (PI/180)
static const float Bovenarm_thetaMin = -1.3962634035f; // -80deg * (PI/180)
static const float phi[N_MOTORS] = {0.0f, 2.0943951024f, 4.1887902048f}; // 0deg, 120deg, 240deg in radialen
//static float jointPosRad[N_MOTORS]; // bovenarmhoeken in radialen (M1,M2,M3)

Bool DeltaKinematics_Inverse(const float tcpPosition_mm[3], float motorRad[N_MOTORS]) // 
{
	//////////////////////////////////////////////////////////////////////////
	// functie declaraties
	//x = tcpPosition_mm[0];
	//y = tcpPosition_mm[1];
	//z = tcpPosition_mm[2];

	//////////////////////////////////////////////////////////////////////////
	// Inverse kinematica: bereken per motor de benodigde bovenarmhoek
	// voor dezelfde TCP-positie. Door de TCP telkens naar het lokale
	// motorvlak te roteren kan dezelfde 2D-berekening voor M1, M2 en M3
	// gebruikt worden.

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
		};*/
		
		// Lokale TCP-positie gezien vanuit motor k.
		//Pk[0] = Rz[0][0] * tcpPosition_mm[0] + Rz[0][1] * tcpPosition_mm[1] + Rz[0][2] * tcpPosition_mm[2]; // lokale x
		//Pk[1] = Rz[1][0] * tcpPosition_mm[0] + Rz[1][1] * tcpPosition_mm[1] + Rz[1][2] * tcpPosition_mm[2]; // lokale y
		//Pk[2] = Rz[2][0] * tcpPosition_mm[0] + Rz[2][1] * tcpPosition_mm[1] + Rz[2][2] * tcpPosition_mm[2]; // lokale z
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