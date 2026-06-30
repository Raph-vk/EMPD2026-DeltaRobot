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
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// file includes
#include "ApplicationTasks.h"
#include "DeltaKinematics.h"
#include "InputHandlerTask.h"
#include "MotionPlanning.h"
#include "PortIOLib.h"
#include "QuadratureCounters.h"
#include "vPrintString.h"
#include "MotorControl.h"

#define PRELOAD_VOLTAGE       (0.5f)
#define PRELOAD_TIME_S        (0.5f)
#define UF_MIN_DISTANCE_MM       (10.0f)

#define USERFRAME_FLASH_PAGE_SIZE   (256U)
#define USERFRAME_FLASH_ADDR        (IFLASH0_ADDR + IFLASH_SIZE - USERFRAME_FLASH_PAGE_SIZE)
#define USERFRAME_FLASH_MAGIC       (0x5546524DU) /* "UFRM" */
#define USERFRAME_FLASH_VERSION     (1U)
#define EEFC_FLASH_KEY              (0x5AU)
#define EEFC_FCMD_EWP               (0x03U)       /* Erase and Write Page */

///////////////////////////////////////////////////////////////////////////////
// private functies
static bool PreloadBacklashTorque(float preloadVoltage, float waitTime_s);
bool UserFrame_PermanentSave(void);

typedef struct
{
	uint32_t magic;
	uint32_t version;
	float originX;
	float originY;
	float cosTheta;
	float sinTheta;
	float zSign;
	uint32_t checksum;
} UserFrameFlashData_t;

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
/*
 * Gebruikt om op gewenste XYZ coördinaten de 
 * ingemeten offset en rotatie toe te passen.
 *
 * Invoer: XYZpos in milimeter, en terug te leveren BaseXYZ
 * Uitvoer: true/false indien gelukt.
*/
bool UserFrame_ToBaseXY(float userX_mm, float userY_mm, float baseZ_mm, float baseXYZ_mm[3])
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
    baseXYZ_mm[0] = userFrameOriginX_mm + (userFrameCosTheta * userX_mm) - (userFrameZSign * userFrameSinTheta * userY_mm);
    baseXYZ_mm[1] = userFrameOriginY_mm + (userFrameSinTheta * userX_mm) + (userFrameZSign * userFrameCosTheta * userY_mm);
    baseXYZ_mm[2] = baseZ_mm; //wordt nog niet mee getransformeerd

    return true;
}


typedef struct
{
    float x;
    float y;
    float z;
} UserFramePoint_t;

typedef enum
{
    TEACH_SETUP = 0,
    TEACH_MOVE_DOWN,
    TEACH_WAIT_POINT,
    TEACH_SAMPLE
} TeachUfStep_t;

static TeachUfStep_t teachStep = TEACH_SETUP;
static uint8_t teachPointIndex = 0U;
static UserFramePoint_t teachPoints[3];

///////////////////////////////////////////////////////////////////////////////
// bool TeachUserframe(...)
/*
 *
*/
bool TeachUserframe(EventBits_t buttonBits)
{
    switch (teachStep)
    {
		// instellen van begin waardes
        case TEACH_SETUP:
        {
            MotionPlanning_RESET();
            teachPointIndex = 0U;

            vPrintString("> Userframe inmeten gestart.\n");
            DisplayInfo_Publish("teachUF: gestart", "Naar meetpositie");

            teachStep = TEACH_MOVE_DOWN;
            break;
        }

        case TEACH_MOVE_DOWN:
        {
			
			// Naar stuk omlaag verplaatsen (verwacht +25deg -> -5deg).
            if (MoveJ_ArmDEG123t(-5.0f, -5.0f, -5.0f, 5.0f))
            {
				// als verplaatsing is voldaan
                MotionPlanning_RESET();

				//motoren spanningsloos maken
                ZetMotorSpanningen(0.0f);
                port_SetBit(BIT_DISABLE_MOTORS, true);

				//info
                vPrintString("> Beweeg naar X0Y0 en druk START.\n");
                DisplayInfo_Publish("UF: meet X0Y0", "Plaats TCP, <START>");

                teachStep = TEACH_WAIT_POINT;
            }
			
            break;
        }
		
		// als spanningsloos wachten tot iets doen
        case TEACH_WAIT_POINT:
        {
			//als resetknop wordt ingedrukt.
            if (buttonBits & EVT_RESET_BUTTON)
            {
                ZetMotorSpanningen(0.0f);
                port_SetBit(BIT_DISABLE_MOTORS, true);

				// true returnen, in controlTask.c wordt terug naar rustPositie verplaatst.
                vPrintString("> Userframe inmeten geannuleerd.\n");
                DisplayInfo_Publish("UF: geannuleerd", "Terug naar READY");
                teachStep = TEACH_SETUP;
                return true;
            }
			
			// inmeten als startknop wordt ingedrukt
            if (buttonBits & EVT_START_BUTTON)
            {
                port_SetBit(BIT_DISABLE_MOTORS, false);
                MotionPlanning_RESET();
                vPrintString("> Userframe preload punt %u.\n", teachPointIndex);
                DisplayInfo_Publish("UF: preload", "Niet bewegen");

                teachStep = TEACH_SAMPLE;
            }
            break;
        }

        case TEACH_SAMPLE:
        {
			// preload voltage toepassen en wachten tot voldaan
			if ( PreloadBacklashTorque(PRELOAD_VOLTAGE, PRELOAD_TIME_S) )
			{
				// indien spanning erop gezet is.
				float motorRad[N_MOTORS] = {0.0f, 0.0f, 0.0f};
				float tcp[3] = {0.0f, 0.0f, 0.0f};

				LeesMotorPositiesRad(motorRad);

				//TCPpositie bepalen tov motorposities
				if (!DeltaKinematics_Forward(motorRad, tcp))
				{
					//indien positie ongeldig is
					ZetMotorSpanningen(0.0f);
					port_SetBit(BIT_DISABLE_MOTORS, true);

					//informatie delen
					vPrintString("> Userframe fout: FK ongeldig.\n");
					DisplayInfo_Publish("UF: fout", "FK ongeldig");

					teachStep = TEACH_SETUP;
					return true;
				}

				//posities opslaan
				teachPoints[teachPointIndex].x = tcp[0];
				teachPoints[teachPointIndex].y = tcp[1];
				teachPoints[teachPointIndex].z = tcp[2];

				// motoren weer spanningsvrij
				ZetMotorSpanningen(0.0f);
				port_SetBit(BIT_DISABLE_MOTORS, true);

				vPrintString("> UF punt %u: X=%.3f Y=%.3f Z=%.3f\n", teachPointIndex, tcp[0], tcp[1], tcp[2]);

				// Naar het volgende te meten punt gaan:
				// 0 -> XY0 is net gemeten, vraag nu X-as punt.
				// 1 -> X-as punt is net gemeten, vraag nu Y-as punt.
				// 2 -> Y-as punt is net gemeten, bereken nu het userframe.
				teachPointIndex++;
				if (teachPointIndex == 1U)
				{
					vPrintString("> Beweeg naar punt op +X-as en druk START.\n");
					DisplayInfo_Publish("UF: meet X-as", "Plaats TCP, START");

					teachStep = TEACH_WAIT_POINT;
				}
				else if (teachPointIndex == 2U)
				{
					vPrintString("> Beweeg naar punt op +Y-as en druk START.\n");
					DisplayInfo_Publish("UF: meet Y-as", "Plaats TCP, START");

					teachStep = TEACH_WAIT_POINT;
				}
				else
				{
					//UF-frame berekenen
					
					// Vector van XY0 naar het gemeten +X-punt.
					// Deze bepaalt de richting van de userframe-X-as in robotbasis.
					float dx = teachPoints[1].x - teachPoints[0].x;
					float dy = teachPoints[1].y - teachPoints[0].y;
					float xDistance = sqrtf((dx * dx) + (dy * dy));

					// Vector van XY0 naar het gemeten +Y-punt.
					// Deze gebruiken we vooral als controle op afstand en richting.
					float yx = teachPoints[2].x - teachPoints[0].x;
					float yy = teachPoints[2].y - teachPoints[0].y;
					float yDistance = sqrtf((yx * yx) + (yy * yy));

					// Beide richtingpunten moeten ver genoeg van XY0 liggen.
					// Anders wordt de richting te gevoelig voor meetruis/speling.
					if ((xDistance < UF_MIN_DISTANCE_MM) || (yDistance < UF_MIN_DISTANCE_MM))
					{
						vPrintString("> Userframe fout: punten te dicht bij X0Y0.\n");
						DisplayInfo_Publish("UF: fout", "Punten te dicht");

						teachStep = TEACH_SETUP;
						return true;
					}

					// X0Y0 wordt de oorsprong van het userframe in robotbasis-coordinaten.
					float newOriginX = teachPoints[0].x;
					float newOriginY = teachPoints[0].y;

					// Normaliseer de X-vector. Dit levert cos/sin van de rotatiehoek op.					
					float newCosTheta = dx / xDistance;
					float newSinTheta = dy / xDistance;

					// Bepaal aan welke kant van de X-as het Y-punt ligt.
					// Omdat de X-vector genormaliseerd is, is crossZ ongeveer de loodrechte
					// afstand van het Y-punt tot de X-as in mm.
					//
					// crossZ > 0: Y-punt ligt links van +X.
					// crossZ < 0: Y-punt ligt rechts van +X.
					// crossZ ~= 0: Y-punt ligt bijna op de X-as, dus ongeldig.
					float crossZ = (newCosTheta * yy) - (newSinTheta * yx);

					// Het Y-punt moet duidelijk naast de X-as liggen.
					// Anders is niet betrouwbaar te bepalen waar de positieve Y-richting zit.
					if (fabsf(crossZ) < UF_MIN_DISTANCE_MM)
					{
						vPrintString("> Userframe fout: Y-punt ligt te veel op X-as.\n");
						DisplayInfo_Publish("UF: fout", "Y-punt ongeldig");

						teachStep = TEACH_SETUP;
						return true;
					}

					// Kies de orientatie van het userframe op basis van het gemeten Y-punt.
					float newZSign = (crossZ >= 0.0f) ? 1.0f : -1.0f;

					// Vanaf hier is alles geldig. Nu pas de actieve userframe-waarden overschrijven.
					userFrameOriginX_mm = newOriginX;
					userFrameOriginY_mm = newOriginY;
					userFrameCosTheta = newCosTheta;
					userFrameSinTheta = newSinTheta;
					userFrameZSign = newZSign;

					vPrintString("> Userframe klaar: origin X=%.3f Y=%.3f cos=%.6f sin=%.6f zSign=%.1f\n",
									userFrameOriginX_mm, userFrameOriginY_mm, userFrameCosTheta, userFrameSinTheta, userFrameZSign);
					DisplayInfo_Publish("UF: klaar", "Frame actief");
					
					// permanent opslaan zodat ook na voeding ontnemen onthouden wordt.
					//if (!UserFrame_PermanentSave())
					//{
					//	DisplayInfo_Publish("UF: fout", "Flash save fout");
					//}

					// Teach-flow opnieuw klaarzetten voor een volgende keer.
					teachStep = TEACH_SETUP;
					return true;
				}
			}//end if-preloads
		}//end-case TEACH_SAMPLE
	}//end-SwitchCase
	return false;
}//end-TeachUserframe



///////////////////////////////////////////////////////////////////////////////
// bool PreloadBacklashTorque(...)
///////////////////////////////////////////////////////////////////////////////
/*
 * Zet kort een vaste open-loop motorspanning op alle armen.
 * Bedoeld om mechanische speling steeds tegen dezelfde kant te drukken voordat
 * een userframe-meetpunt wordt gesampled.
 */
static bool PreloadBacklashTorque(float preloadVoltage, float waitTime_s)
{
    static bool setupDone = false;
	static float tau = 0.0f; //actuele tijd
	static const float Ts = 0.001f; //1kHz regelTick (externe clock)

	// minimale procestijd benodigd
    if (waitTime_s <= 0.0f)
    {
        return false;
    }

	//eenmalig instellingen doen
    if (!setupDone)
    {
        tau = 0.0f;
        setupDone = true;
    }

	ZetMotorSpanningen(preloadVoltage);

	// als tijd verstreken is
    if (tau >= waitTime_s)
    {
        setupDone = false;
        tau += Ts;
        return true;
    }

    tau += Ts;
    return false;
}



///////////////////////////////////////////////////////////////////////////////
// static uint32_t UserFrame_CalcChecksum(...)
/*
 * Berekent een simpele XOR-checksum over de flash-struct.
 * Het checksum-veld zelf wordt niet meegenomen.
 */
static uint32_t UserFrame_CalcChecksum(const UserFrameFlashData_t *data)
{
	const uint32_t *words = (const uint32_t *)data;
	uint32_t checksum = 0U;
	uint32_t wordCount = (uint32_t)(sizeof(UserFrameFlashData_t) / sizeof(uint32_t));

	for (uint32_t i = 0U; i < (wordCount - 1U); i++)
	{
		checksum ^= words[i];
	}

	return checksum;
}

///////////////////////////////////////////////////////////////////////////////
// static bool UserFrame_LoadPermanent(...)
/*
 * Leest de flashstruct uit en laast deze waardes op.
 */
bool UserFrame_LoadPermanent(void)
{
	const UserFrameFlashData_t *data =
	(const UserFrameFlashData_t *)USERFRAME_FLASH_ADDR;

	if (data->magic != USERFRAME_FLASH_MAGIC)
	{
		return false;
	}

	if (data->version != USERFRAME_FLASH_VERSION)
	{
		return false;
	}

	if (data->checksum != UserFrame_CalcChecksum(data))
	{
		return false;
	}

	userFrameOriginX_mm = data->originX;
	userFrameOriginY_mm = data->originY;
	userFrameCosTheta = data->cosTheta;
	userFrameSinTheta = data->sinTheta;
	userFrameZSign = data->zSign;

	vPrintString("> Userframe geladen uit flash.\n");
	return true;
}

///////////////////////////////////////////////////////////////////////////////
// static bool UserFrame_FlashWaitReady(...)
/*
 * Wacht tot flash bank 1 klaar is en controleert command-/lock-fouten.
 * De laatste flash-page van de Arduino Due ligt in flash bank 1.
 */
RAMFUNC
static bool UserFrame_FlashWaitReady(void)
{
	while ((REG_EFC1_FSR & EEFC_FSR_FRDY) == 0U)
	{
		/* wachten tot de flash-controller klaar is */
	}

	return ((REG_EFC1_FSR & (EEFC_FSR_FCMDE | EEFC_FSR_FLOCKE)) == 0U);
}

///////////////////////////////////////////////////////////////////////////////
// bool UserFrame_PermanentSave(void)
/*
 * Slaat het actieve userframe op in de laatste interne flash-page.
 * Let op: reserveer deze page in het linker script, bijvoorbeeld door de
 * rom-lengte 256 bytes korter te maken. Anders kan hier programmacode staan.
 */
RAMFUNC
bool UserFrame_PermanentSave(void)
{
	UserFrameFlashData_t data;
	uint32_t pageBuffer[USERFRAME_FLASH_PAGE_SIZE / sizeof(uint32_t)];
	volatile uint32_t *flashWords = (volatile uint32_t *)USERFRAME_FLASH_ADDR;
	uint32_t pageNumber = (USERFRAME_FLASH_ADDR - IFLASH1_ADDR) / IFLASH1_PAGE_SIZE;

	data.magic = USERFRAME_FLASH_MAGIC;
	data.version = USERFRAME_FLASH_VERSION;
	data.originX = userFrameOriginX_mm;
	data.originY = userFrameOriginY_mm;
	data.cosTheta = userFrameCosTheta;
	data.sinTheta = userFrameSinTheta;
	data.zSign = userFrameZSign;
	data.checksum = UserFrame_CalcChecksum(&data);

	for (uint32_t i = 0U; i < (USERFRAME_FLASH_PAGE_SIZE / sizeof(uint32_t)); i++)
	{
		pageBuffer[i] = 0xFFFFFFFFU;
	}

	memcpy(pageBuffer, &data, sizeof(data));

	if (!UserFrame_FlashWaitReady())
	{
		vPrintString("> Userframe flash niet klaar voor schrijven.\n");
		return false;
	}

	for (uint32_t i = 0U; i < (USERFRAME_FLASH_PAGE_SIZE / sizeof(uint32_t)); i++)
	{
		flashWords[i] = pageBuffer[i];
	}

	REG_EFC1_FCR =
		EEFC_FCR_FKEY(EEFC_FLASH_KEY) |
		EEFC_FCR_FARG(pageNumber) |
		EEFC_FCR_FCMD(EEFC_FCMD_EWP);

	if (!UserFrame_FlashWaitReady())
	{
		vPrintString("> Userframe opslaan in flash mislukt.\n");
		return false;
	}

	vPrintString("> Userframe opgeslagen in flash.\n");
	return true;
}
