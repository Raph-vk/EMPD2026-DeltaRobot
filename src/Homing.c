/*
 * Homing.c
 *
 * Created: 2026
 * Author : Raph van Koeveringe
 *
 * Doel van deze module
 * ---------------------
 * Deze module bevat de homing-cyclus van de delta robot.
 *
 * Belangrijk uitgangspunt
 * ----------------------
 * homeAllMotors() wordt aangeroepen vanuit een 1 kHz regel-/control-loop.
 * Elke aanroep voert één kleine stap uit. Er staan daarom geen blocking delays,
 * while-wachtlussen of taskSleep() functies in deze module.
 *
 * Bestandsverdeling
 * -----------------
 * Homing.c       : bepaalt WAT de homing-procedure doet.
 * MotorControl.c : bepaalt HOE motoren, encoders, DAC's en switches worden
 *                  aangestuurd of uitgelezen.
 */

#include "Homing.h"

///////////////////////////////////////////////////////////////////////////////
// SYSTEM INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include <asf.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////////
// PROJECT INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include "vPrintString.h"
#include "PortIOLib.h"

#include "MotorControl.h" //<-- helper functies motor-aansturing
#include "QuadratureCounters.h" //<-- Lees motorposities

#include "ApplicationTasks.h"
#include "ControlTask.h"
#include "MachinePins.h"
#include "Regelaar.h"



///////////////////////////////////////////////////////////////////////////////
// HOMING INSTELLINGEN
///////////////////////////////////////////////////////////////////////////////
#define Tsample						(0.001f) // 1ms, bij 1kHz clock.
#define DEG_TO_RAD                  (0.01745329252f)
#define RAD_TO_DEG					(57.2957795131f)

#define HOME_RAD				    (0.0f)
#define BACKOFF_RAD					(5.0f * DEG_TO_RAD)

//BIJV: 10 deg/s * 0.001 s = 0.01 graden per regelstap (bij 1kHz clock)
#define GROF_ZOEK_SNELHEID_DEG_S    (25.0f)
#define GROF_ZOEK_SNELHEID_RAD_S    (GROF_ZOEK_SNELHEID_DEG_S * DEG_TO_RAD)

#define FIJN_ZOEK_SNELHEID_DEG_S    (5.0f)
#define FIJN_ZOEK_SNELHEID_RAD_S    (FIJN_ZOEK_SNELHEID_DEG_S * DEG_TO_RAD)

/*
 * Een korte wachttijd voordat de nauwkeurige homing begint.
 *
 * Omdat homeAllMotors() op 1 kHz wordt aangeroepen: 1000 ticks = 1000 ms = 1 seconde.
 */
#define STABILISATIE_TIJD_MS         (1000U)

///////////////////////////////////////////////////////////////////////////////
// TYPEDEFINITIES
///////////////////////////////////////////////////////////////////////////////

typedef enum
{
	HOMING_IDLE = 0,
	HOMING_GROF_ZOEKEN,
	HOMING_NAAR_BACKOFF,
	HOMING_NAUW_ZOEKEN,
	HOMING_KLAAR
} HomingStatus;

///////////////////////////////////////////////////////////////////////////////
// PRIVATE VARIABELEN
///////////////////////////////////////////////////////////////////////////////
static HomingStatus homingStatus = HOMING_IDLE;

/*
 * Gewenste armposities tijdens homing.
 * Alle waarden zijn armhoeken in radialen.
 */
static float armDoelRad[N_MOTORS] = { 0.0f, 0.0f, 0.0f };

/*
 * Buffer voor gemeten motorhoeken.
 * MotorControl leest motorposities uit in radialen op de motoras.
 */
static float armPositieRad[N_MOTORS] = { 0.0f, 0.0f, 0.0f };

/*
 * Per motor wordt bijgehouden of de grove, backOffPos of fijne homing al gevonden zijn.
 */
static bool grofGevonden[N_MOTORS] = { false, false, false };
static bool armOpBackoffPos[N_MOTORS] = { false, false, false };
static bool fijnGevonden[N_MOTORS] = { false, false, false };


static uint32_t stabilisatieTeller = 0U;
static bool offsetZeroRequested = false;

static float uDac = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

static bool AlleFlagsWaar(const bool flags[N_MOTORS]);
static bool RampArmDoelNaar(uint8_t motor, float doelRad, float snelheidRadS);
static float bepaalFoutOpMotor(float doelArmRad, float gemetenArmRad);

static void Initialiseren(void);
static void GrofZoeken(void);
static void NaarBackoffPositie(void);
static void NauwZoeken(void);
static void Afronden(void);
static void FoutAfhandeling(void);

///////////////////////////////////////////////////////////////////////////////
// publieke functies
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// bool homeAllMotors(void)
/*
 * Voert de homing-procedure stap voor stap uit vanuit de 1 kHz control-loop.
 * Invoer: geen.
 * Uitvoer: true wanneer alle motoren klaar zijn met homing, anders false.
 */
bool homeAllMotors(void)
{
	if (homingStatus == HOMING_IDLE)
	{
		Initialiseren();
		homingStatus = HOMING_GROF_ZOEKEN;
		return false;
	}

	if (homingStatus == HOMING_GROF_ZOEKEN)
	{
		GrofZoeken();
		return false;
	}

	if (homingStatus == HOMING_NAAR_BACKOFF)
	{
		NaarBackoffPositie();
		return false;
	}

	if (homingStatus == HOMING_NAUW_ZOEKEN)
	{
		NauwZoeken();
		return false;
	}

	if (homingStatus == HOMING_KLAAR)
	{
		Afronden();
		return true;
	}

	FoutAfhandeling();
	return false;
}

///////////////////////////////////////////////////////////////////////////////
// void resetHoming(void)
/*
 *
 *
*/
void resetHoming(void)
{
	homingStatus = HOMING_IDLE;
	stabilisatieTeller = 0U;
	offsetZeroRequested = false;
	uDac = 0.0f;

	for (uint8_t motor = 0U; motor < N_MOTORS; motor++)
	{
		armDoelRad[motor] = 0.0f;
		armPositieRad[motor] = 0.0f;

		grofGevonden[motor] = false;
		armOpBackoffPos[motor] = false;
		fijnGevonden[motor] = false;

		removeRegelaarHistory(motor);
	}
}

///////////////////////////////////////////////////////////////////////////////
// PRIVATE HELPER FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// static bool AlleFlagsWaar(const bool flags[N_MOTORS])
/*
 * Controleert of alle vlaggen in een motor-array true zijn.
 * Invoer: flags bevat per motor een statusvlag.
 * Uitvoer: true wanneer alle vlaggen true zijn, anders false.
 */
static bool AlleFlagsWaar(const bool flags[N_MOTORS])
{
    for (uint8_t motor = 0U; motor < N_MOTORS; motor++)
    {
        if (flags[motor] == false)
        {
            return false;
        }
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// static bool RampArmDoelNaar(uint8_t motor, float doelRad, float snelheidRadS)
/*
 * Verplaatst de gewenste armpositie incrementeel naar een doelhoek.
 * Invoer: motor kiest de arm, doelRad is de doelhoek en snelheidRadS de rampsnelheid.
 * Uitvoer: true wanneer het doel bereikt is, anders false.
 */
static bool RampArmDoelNaar(uint8_t motor, float doelRad, float snelheidRadS)
{
    const float stapRad = fabsf(snelheidRadS) * Tsample;
    const float foutRad = doelRad - armDoelRad[motor];

    if (fabsf(foutRad) <= stapRad)
    {
        armDoelRad[motor] = doelRad;
        return true;
    }

    if (foutRad > 0.0f)
    {
        armDoelRad[motor] += stapRad;
    }
    else
    {
        armDoelRad[motor] -= stapRad;
    }

    return false;
}

///////////////////////////////////////////////////////////////////////////////
// static float bepaalFoutOpMotor(float doelArmRad, float gemetenArmRad)
/*
 * Berekent de regelfout op de motoras vanuit armhoeken.
 * Invoer: doelArmRad en gemetenArmRad zijn armhoeken in radialen.
 * Uitvoer: fout in motor-radialen, inclusief overbrengingsverhouding.
 */
//static uint32_t i=0;
static float bepaalFoutOpMotor(float doelArmRad, float gemetenArmRad)
{
    /*
     * Unit-afspraak:
     * Homing bewaart armDoelRad en armPositieRad in arm-radialen.
     * PIDregelaar() krijgt dezelfde eenheid als MotionEngine.c: motor-radialen.
     */
    return (doelArmRad - gemetenArmRad) * i_twk;
}

///////////////////////////////////////////////////////////////////////////////
// PRIVATE HOMING STATE HANDLERS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// static void Initialiseren(void)
/*
 * Zet de homing-status, doelposities en encodertellers klaar voor een nieuwe cyclus.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; interne homing-variabelen worden gereset.
 */
static void Initialiseren(void)
{
    vPrintString("> Homing gestart.\n");
	
	//Reset flags en teller
	stabilisatieTeller = 0U;
	offsetZeroRequested = false;
    for (uint8_t motor = 0U; motor < N_MOTORS; motor++)
    {
	    grofGevonden[motor] = false;
	    armOpBackoffPos[motor] = false;
	    fijnGevonden[motor] = false;
	    armDoelRad[motor] = 0.0f;
    }
	DisplayInfo_Publish("Home: grof zoeken", "Zoekt referenties");
	
	//niet essentieël, wel prettig.
	uDac = 0.0f;
    //Initialiseer encoder teller
    QCEncodersSetup();

    
    // Absolute startpositie is onbekend. Daarom worden de tellers hier eerst op nul gezet.
    // Vanaf dit moment wordt alleen relatief gezocht.
    EncodersClearCount();
	
	//Motorspanning mag nu
	port_SetBit(BIT_DISABLE_MOTORS, false);
		
	vPrintString("> Initialiseren voldaan.\n");
} //eind Initialiseren();


///////////////////////////////////////////////////////////////////////////////
// static void GrofZoeken(void)
/*
 * Zoekt per motor grof naar de home-switch en nult de encoder bij detectie.
 * Invoer: geen directe parameters; gebruikt actuele switches en encoderposities.
 * Uitvoer: geen returnwaarde; stuurt motoren aan en zet de volgende homing-state.
 */
static void GrofZoeken(void)
{
    /*
     * Doel van deze state:
     * - De armreferentie loopt met een continue snelheid omhoog.
     * - De PIDregelaar volgt deze oplopende referentie.
     * - Zodra de home-switch geraakt wordt, wordt de encoder op nul gezet.
     *
     * Deze functie wordt elke 1 ms aangeroepen.
     */

    LeesArmPositiesRad(armPositieRad);

	//voor iedere arm
    for (uint8_t motor = 0; motor < N_MOTORS; motor++)
    {
		//Als homeSwitch al gevonden is, rustig naar BACKOFF Positie bewegen.
		if (grofGevonden[motor])
		{
			RampArmDoelNaar(motor, BACKOFF_RAD, FIJN_ZOEK_SNELHEID_RAD_S);
		}
		//Als home-switch actief is
        else if (motor_IsHomeLimitActive(motor))
        {
			EncoderClearCount(motor);

            grofGevonden[motor] = true;
            armDoelRad[motor] = HOME_RAD;
			armPositieRad[motor] = HOME_RAD;   // important
            vPrintString("> Motor %u: grove homing gevonden, encoder genuld.\n", (unsigned int)motor);
        }
		//Als homeSwitch nog niet gevonden is
		else
		{
			// Met constante snelheid de referentie hoekpositie verhogen
			RampArmDoelNaar(motor, -INFINITY, GROF_ZOEK_SNELHEID_RAD_S);
		}
        
		//De motorspanning met de fout bepalen en uitsturen.
		uDac = PIDregelaar(motor, bepaalFoutOpMotor(armDoelRad[motor], armPositieRad[motor]));
		ZetMotorSpanning(motor, uDac);
    }

	// Als alle armen grof gerefereerd zijn,
    if (AlleFlagsWaar(grofGevonden))
    {
        vPrintString("> Groffe homing klaar, Wacht tot alle armen op terugtrekpositie zijn.\n");
        stabilisatieTeller = 0;
		DisplayInfo_Publish("Home: backoff", "Armen terugtrekken");
		homingStatus = HOMING_NAAR_BACKOFF;
	}
} // eind GrofZoeken();

///////////////////////////////////////////////////////////////////////////////
// static void NaarBackoffPositie(void)
/*
 * Beweegt alle armen naar de backoff-positie en wacht tot ze stabiel staan.
 * Invoer: geen directe parameters; gebruikt gemeten armposities.
 * Uitvoer: geen returnwaarde; schakelt door naar nauwkeurig zoeken zodra alles klaar is.
 */
static void NaarBackoffPositie(void)
{
    /*
     * Doel van deze state:
     * - Alle armen naar backoff positie bewegen.
     * - Daarna kort stabiliseren.
     * - Vervolgens starten met nauwkeurige homing terug naar de switch.
    */
	LeesArmPositiesRad(armPositieRad);
	
	//voor iedere motor
	for (uint8_t motor = 0; motor < N_MOTORS; motor++)
	{
		//armDoel naar BACKOFF Positie bewegen.
		if (RampArmDoelNaar(motor, BACKOFF_RAD, FIJN_ZOEK_SNELHEID_RAD_S) )
		{
			//arm is (ECHT) op positie als gemeten binnen 0.5 graad van gewenste positie
			armOpBackoffPos[motor] = ( fabsf(armPositieRad[motor] - BACKOFF_RAD) <= (0.5 * DEG_TO_RAD));
		}
		uDac = PIDregelaar(motor, bepaalFoutOpMotor(armDoelRad[motor], armPositieRad[motor]));
		ZetMotorSpanning(motor, uDac);
	}

	// Als op positie even stabiliseren en offset nullen
	if (AlleFlagsWaar(armOpBackoffPos))
	{
		//Armen nog even uitstabiliseren op hoek.
		stabilisatieTeller++;
		if (stabilisatieTeller > STABILISATIE_TIJD_MS)
		{
			// #if TCP_COMP == 0
			//  offsetZeroRequested = false;
			//  stabilisatieTeller = 0U;
			//  vPrintString("> TCP-comp uit. Offset nullen overgeslagen. Start nauwkeurig zoeken.\n");
			//  homingStatus = HOMING_NAUW_ZOEKEN;
			//  return;
			// #endif
			
			
			// Vragen of offsetmeting genult wordt.
			if (!offsetZeroRequested)
			{
				vPrintString("> Backoff gestabiliseerd. Offset meting nullen.\n");
				if (handle_OffsetZeroDone != NULL && handle_OffsetZeroRequest != NULL)
				{
					xSemaphoreTake(handle_OffsetZeroDone, 0);      // oude done wissen
					xSemaphoreGive(handle_OffsetZeroRequest);      // zeroing starten
					offsetZeroRequested = true;
				}
				DisplayInfo_Publish("Offset: nullen", "Wacht op meting");
				return;
			}
			//Wachten tot offsetmeting gereed is
			if (handle_OffsetZeroDone != NULL)
			{
				if (xSemaphoreTake(handle_OffsetZeroDone, 0) == pdTRUE)
				{
					offsetZeroRequested = false;
					stabilisatieTeller = 0;
					vPrintString("> OffsetMeting genult. Start nauwkeurig zoeken.\n");
					DisplayInfo_Publish("Home: fijn zoeken", "Nauwkeurig nullen");
					homingStatus = HOMING_NAUW_ZOEKEN;
				}
			}
		}
	}//end stabilisatie
} //eind NaarBackoffPositie();

///////////////////////////////////////////////////////////////////////////////
// static void NauwZoeken(void)
/*
 * Beweegt langzaam terug naar de home-switch voor nauwkeurige nulstelling.
 * Invoer: geen directe parameters; gebruikt home-switches en encoderposities.
 * Uitvoer: geen returnwaarde; zet de homing-status op klaar wanneer alle motoren gevonden zijn.
 */
static void NauwZoeken(void)
{
    /*
     * Doel van deze state:
     * - Vanaf de backoff-positie langzaam terug bewegen richting home-switch.
     * - De armreferentie loopt met FIJN_ZOEK_SNELHEID_DEG_S terug naar HOME_RAD.
     * - Zodra de switch geraakt wordt, wordt de encoder definitief genuld.
     *
     * Let op:
     * Deze functie wordt elke 1 ms aangeroepen.
     * Er staat dus geen while-loop, taskSleep() of blocking delay in.
     */

    LeesArmPositiesRad(armPositieRad);

    for (uint8_t motor = 0U; motor < N_MOTORS; motor++)
    {
		// Als homeswitch bereikt is,
		if (motor_IsHomeLimitActive(motor) && !fijnGevonden[motor])
        {
			//definitieve/nauwkeurige nul
            EncoderClearCount(motor);

            fijnGevonden[motor] = true;
            armDoelRad[motor] = HOME_RAD;
			armPositieRad[motor] = HOME_RAD;

            vPrintString("> Motor %u: nauwkeurige homing gevonden, encoder definitief genuld.\n",
                         (unsigned int)motor);
        }
		else if (fijnGevonden[motor])
		{
			armDoelRad[motor] = HOME_RAD;
		}
        else
        {
            //Vanaf BACKOFF_RAD langzaam terug richting HOME_RAD bewegen.
			RampArmDoelNaar(motor, -INFINITY, FIJN_ZOEK_SNELHEID_RAD_S);
        }

		//Positie regeling aansturen
		uDac = PIDregelaar(motor, bepaalFoutOpMotor(armDoelRad[motor], armPositieRad[motor]));
		ZetMotorSpanning(motor, uDac);
    }

    if (AlleFlagsWaar(fijnGevonden))
    {
        vPrintString("> Nauwkeurige homing klaar.\n");
        homingStatus = HOMING_KLAAR;
    }

} //eind NauwZoeken();

///////////////////////////////////////////////////////////////////////////////
// static void Afronden(void)
/*
 * Houdt de motoren op de homepositie en ruimt de homing-status op.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; reset interne homing-variabelen voor een volgende cyclus.
 */
static void Afronden(void)
{
	LeesArmPositiesRad(armPositieRad);

	for (uint8_t motor = 0U; motor < N_MOTORS; motor++)
	{		
		//Positie regeling aansturen
		armDoelRad[motor] = HOME_RAD;
		uDac = PIDregelaar(motor, bepaalFoutOpMotor(armDoelRad[motor], armPositieRad[motor]));
		ZetMotorSpanning(motor, uDac);
		
		//Direct alles resetten voor volgende Homing
        armDoelRad[motor] = 0.0f;
        armPositieRad[motor] = 0.0f;

        grofGevonden[motor] = false;
        armOpBackoffPos[motor] = false;
        fijnGevonden[motor] = false;		
	}
	stabilisatieTeller = 0U;
	uDac = 0.0f;
	homingStatus = HOMING_IDLE;
} //eind Afronden();


///////////////////////////////////////////////////////////////////////////////
// static void FoutAfhandeling(void)
/*
 * Stopt de motoruitgangen wanneer homing in een foutpad belandt.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; zet het systeem terug naar FAULT.
 */
static void FoutAfhandeling(void)
{
	ZetMotorSpanningen(0.0f);
	vPrintString("> Homing gestopt: naar FAULT.\n");
	
	homingStatus = HOMING_IDLE;
	ToState(STATE_FAULT);
} //eind FoutAfhandeling();
