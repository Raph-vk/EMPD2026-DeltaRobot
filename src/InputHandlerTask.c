/*
 * InputHandlerTask.c
 *
 * Created: 10/04/2023
 *  Author: Raph van Koeveringe
 */ 
#include "InputHandlerTask.h"
///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board
#include "bits.h"
#include "ADCLib.h"
#include "PortIOLib.h"
#include "SwitchLib.h"

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes
#include "vPrintString.h"
#include "TaskSleep.h"


///////////////////////////////////////////////////////////////////////////////
// application includes
#include "MachinePins.h"
#include "ApplicationTasks.h"

///////////////////////////////////////////////////////////////////////////////
// defines

//analoge offset-meting sensoren
#define xDisturbanceChannel		(3U)
#define yDisturbanceChannel		(4U)

#define adcMaxValue				(4095.0f)
#define mmStroke				(30.43f)
#define mmThreshold				(0.05f)
#define calibrationSamples		(100U)

/*
 * Vaste EMA-filterfactor voor de offsetcompensatie.
 *
 * Lager geeft meer rust maar meer vertraging. 
 * Hoger reageert sneller, maar laat meer ADC-ruis door.
 */
#define OFFSET_FILTER_ALPHA			(0.35f)

// STOP-knop lang vasthouden om bewust userframe teach aan te vragen.
#define STOP_LONG_PRESS_TICKS		(5000U)
#define STOP_DEBOUNCE_TICKS			(20U)

///////////////////////////////////////////////////////////////////////////////
//#define ADC_REFERENCE_VOLTAGE         (3.3f)
//#define CURRENT_SENSOR_VOLTS_PER_AMP  (0.1f)
//#define stroomChannel (2U)
//static bool hasCurrentSample = false;
//static uint32_t vorigeStroomData = 0;
//static float zeroCurrentVoltage = 2.5f;

///////////////////////////////////////////////////////////////////////////////
// static bool IsButtonPressed(uint8_t pcbSwitch, uint8_t inputBit)
/*
 * Controleert of een knop actief is via de PCB-switch of externe input.
 * Invoer: pcbSwitch is de lokale switch, inputBit is de externe inputlijn.
 * Uitvoer: true wanneer een van beide ingangen actief is, anders false.
 */
static bool IsButtonPressed(uint8_t pcbSwitch, uint8_t inputBit)
{
	return switch_IsPressed(pcbSwitch) || !port_IsBitSet(inputBit);
}

///////////////////////////////////////////////////////////////////////////////
// static bool ButtonWasPressed(uint8_t pcbSwitch, uint8_t inputBit)
/*
 * Verwerkt debounce en wacht tot de knop weer losgelaten is.
 * Invoer: pcbSwitch is de lokale switch, inputBit is de externe inputlijn.
 * Uitvoer: true wanneer een geldige knopdruk is afgerond, anders false.
 */
static bool ButtonWasPressed(uint8_t pcbSwitch, uint8_t inputBit)
{
	//Als knop niet ingedrukt is
	if (!IsButtonPressed(pcbSwitch, inputBit))
	{
		return false;
	}

	//debounce
	taskSleep(20);

	//Als niet meer ingedrukt is
	if (!IsButtonPressed(pcbSwitch, inputBit))
	{
		return false;
	}

	//Wacht tot knop niet meer ingedrukt wordt.
	while (IsButtonPressed(pcbSwitch, inputBit))
	{
		taskSleep(1);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
// static void ProcessStopButton(void)
/*
 * Verwerkt de STOP-knop zonder blokkeren.
 * Korte druk blijft een normaal STOP-event na loslaten.
 * Lang vasthouden geeft een apart TEACH-event en onderdrukt daarna het STOP-event.
 */
static void ProcessStopButton(void)
{
	static bool stopWasPressed = false;
	static bool teachEventSent = false;
	static uint16_t stopPressTicks = 0U;

	const bool stopPressed = IsButtonPressed(PCB_SWITCH_STOP, BIT_STOP);

	if (stopPressed)
	{
		if (!stopWasPressed)
		{
			stopWasPressed = true;
			teachEventSent = false;
			stopPressTicks = 0U;
		}

		if (stopPressTicks < STOP_LONG_PRESS_TICKS)
		{
			stopPressTicks++;
		}

		if ((stopPressTicks >= STOP_LONG_PRESS_TICKS) && !teachEventSent)
		{
			vPrintString("> STOP lang ingedrukt: TEACH aangevraagd.\n");
			xEventGroupSetBits(handle_ButtonEventGroup, EVT_TEACH_BUTTON);
			teachEventSent = true;
		}

		return;
	}

	if (stopWasPressed)
	{
		if (!teachEventSent && (stopPressTicks >= STOP_DEBOUNCE_TICKS))
		{
			vPrintString("> STOP button is pressed!\n");
			xEventGroupSetBits(handle_ButtonEventGroup, EVT_STOP_BUTTON);
		}

		stopWasPressed = false;
		teachEventSent = false;
		stopPressTicks = 0U;
	}
}

///////////////////////////////////////////////////////////////////////////////
// static uint16_t Median3(...)
/*
 * Geeft de middelste waarde van drie ADC-samples terug.
 * Dit haalt losse pieken weg met slechts ongeveer een sample vertraging.
 */
static uint16_t Median3(uint16_t a, uint16_t b, uint16_t c)
{
	uint16_t temp;

	if (a > b)
	{
		temp = a;
		a = b;
		b = temp;
	}
	if (b > c)
	{
		temp = b;
		b = c;
		c = temp;
	}
	if (a > b)
	{
		temp = a;
		a = b;
		b = temp;
	}

	return b;
}

///////////////////////////////////////////////////////////////////////////////
// static OffsetPos_t FilterOffsetMeasurement(...)
/*
 * Filtert de offsetmeting in twee stappen:
 *
 * 1. Median-of-3 op ruwe ADC-waarden.
 *    Dit pakt drie opeenvolgende samples en kiest de middelste waarde. Een losse
 *    piek omhoog of omlaag wordt daardoor genegeerd, zonder veel vertraging.
 *
 * 2. Vaste EMA op de offset in mm.
 *    De EMA maakt geen gemiddelde van een grote buffer. Daardoor blijft de
 *    reactie snel. Per sample schuift de gefilterde waarde een percentage op
 *    richting de nieuwe meting. Dat percentage is de vaste OFFSET_FILTER_ALPHA.
 */
static void FilterOffsetMeasurement(OffsetPos_t *outMeasurement, uint16_t xAdcRaw, uint16_t yAdcRaw, const OffsetPos_t *zeroPos, bool resetFilter)
{
	static uint16_t xAdcSamples[3] = {0U, 0U, 0U};
	static uint16_t yAdcSamples[3] = {0U, 0U, 0U};
	static uint8_t medianSampleIndex = 0U;
	static OffsetPos_t filteredMeasurement = {0.0f, 0.0f};

	//pointer check of ze wel bestaan
	if ((outMeasurement == NULL) || (zeroPos == NULL))
	{
		return;
	}

	// vertagingswaardes invullen bij reset
	if (resetFilter)
	{
		for (uint8_t i = 0U; i < 3U; i++)
		{
			xAdcSamples[i] = xAdcRaw;
			yAdcSamples[i] = yAdcRaw;
		}

		medianSampleIndex = 0U;
	}
	//anders oudste waarde herschrijven
	else
	{
		xAdcSamples[medianSampleIndex] = xAdcRaw;
		yAdcSamples[medianSampleIndex] = yAdcRaw;

		medianSampleIndex++;
		if (medianSampleIndex >= 3U)
		{
			medianSampleIndex = 0U;
		}
	}

	// Median-of-3: gebruik de middelste van de laatste drie ruwe ADC-samples.
	const uint16_t xAdcMedian = Median3(xAdcSamples[0], xAdcSamples[1], xAdcSamples[2]);
	const uint16_t yAdcMedian = Median3(yAdcSamples[0], yAdcSamples[1], yAdcSamples[2]);

	//ADC-count omrekenen naar milimeters
	OffsetPos_t rawMeasurement;
	rawMeasurement.x = (((float)xAdcMedian - zeroPos->x) / adcMaxValue) * mmStroke;
	rawMeasurement.y = (((float)yAdcMedian - zeroPos->y) / adcMaxValue) * mmStroke;

	if (resetFilter)
	{
		// Eerste sample na opstarten/nullen: direct overnemen.
		filteredMeasurement = rawMeasurement;
		*outMeasurement = filteredMeasurement;
		return;
	}

	// EMA-adaptive filter
	// filtered = filtered + alpha * delta
	filteredMeasurement.x += OFFSET_FILTER_ALPHA * (rawMeasurement.x - filteredMeasurement.x);
	filteredMeasurement.y += OFFSET_FILTER_ALPHA * (rawMeasurement.y - filteredMeasurement.y);

	*outMeasurement = filteredMeasurement;
}


///////////////////////////////////////////////////////////////////////////////
// static void ProcessCurrentSensorData(uint32_t stroomData)
//static bool hasCurrentSample = false;
//static uint32_t vorigeStroomData = 0;
//static float zeroCurrentVoltage = 2.5f;        // Better: measure this during startup
/*
 * Zet de ADC-waarde van de stroomsensor om naar een stroomwaarde in ampere.
 * Invoer: stroomData is de ruwe ADC-waarde van de sensor.
 * Uitvoer: geen returnwaarde; schrijft de berekende stroom naar handle_stroomQueue.
 */
/*
static void ProcessCurrentSensorData(uint32_t stroomData)
{
	uint32_t verschil;

	if (!hasCurrentSample)
	{
		vorigeStroomData = stroomData;
		hasCurrentSample = true;
	}
	else
	{
		if (stroomData >= vorigeStroomData)
		{
			verschil = stroomData - vorigeStroomData;
		}
		else
		{
			verschil = vorigeStroomData - stroomData;
		}

		if (verschil < 100u)
		{
			return;
		}

		vorigeStroomData = stroomData;
	}

	float spanning = ((float)stroomData * ADC_REFERENCE_VOLTAGE) / (float)ADC_MAX_VALUE;

	// ACS712: uitgang is rond VCC/2 bij 0A
	float stroom = (spanning - zeroCurrentVoltage) * 10.0f; // *10.0f is gelijk aan delen door 100mV/A
	xQueueOverwrite(handle_stroomQueue, &stroom);
}*/

///////////////////////////////////////////////////////////////////////////////
// static void ProcessOffsetPositionData(uint16_t xAdcRaw, uint16_t yAdcRaw)
//static uint32_t printTEL = 0;
static void ProcessOffsetPositionData(uint16_t xAdcRaw, uint16_t yAdcRaw)
{
	//static wordt eenmalig aangemaakt
	static OffsetPos_t previousMeasurement = { INFINITY, INFINITY }; //oneindig zodat waarde altijd te groot is voor treshold
	static OffsetPos_t measurement = { 0.0f, 0.0f };
	static OffsetPos_t ZeroPos = {0.0f, 0.0f};

	static uint32_t xSum = 0, ySum = 0;
	static uint8_t sampleCount = 0;

	static bool zeroingActive = true;
	static bool offsetGehomed = true;

	
	// Als zero wordt aangevraagd, TAKE
	// "zeroingActive" flag op true zetten
	// en variabelen goed zetten.
	if (handle_OffsetZeroRequest != NULL)
	{
		if (xSemaphoreTake(handle_OffsetZeroRequest, 0) == pdTRUE)
		{
			zeroingActive = true;
			offsetGehomed = false;
			sampleCount = 0;
			xSum = 0;
			ySum = 0;
			
			//voor zekerheid óók taken
			if (handle_OffsetZeroDone != NULL)
			{
				xSemaphoreTake(handle_OffsetZeroDone, 0); 
			}
			vPrintString("> Offset nullen requested.\n");
		}
	}

	// als offset positie genult moet worden.
	if (zeroingActive)
	{
		// uitlezen van data en toevoegen.
		xSum += xAdcRaw;
		ySum += yAdcRaw;
		sampleCount++;
	
		// als er genoeg samples gevonden zijn
		if (sampleCount >= calibrationSamples)
		{
			vPrintString("> Offset calibratie klaar.\n");
			
			ZeroPos.x = (float)xSum / (float)calibrationSamples;
			ZeroPos.y = (float)ySum / (float)calibrationSamples;
			previousMeasurement = (OffsetPos_t){ INFINITY, INFINITY };			

			zeroingActive = false;
			offsetGehomed = true;

			// nieuwe waarde naar queue schrijven
			FilterOffsetMeasurement(&measurement, xAdcRaw, yAdcRaw, &ZeroPos, true);
			if (handle_OffsetQueue != NULL)
			{
				if (xQueueOverwrite(handle_OffsetQueue, &measurement) == pdPASS)
				{
					previousMeasurement = measurement;
				}
			}
			//vrij geven dat nullen voldaan is.
			if (handle_OffsetZeroDone != NULL)
			{
				xSemaphoreGive(handle_OffsetZeroDone);
			}
		}
	}
	else if (offsetGehomed)
	{
		FilterOffsetMeasurement(&measurement, xAdcRaw, yAdcRaw, &ZeroPos, false);
	
		// Bepaal het verschil met de vorige meting die naar de queue is geschreven.
		// Als X of Y genoeg veranderd is, naar de queue schrijven.
		if ( (fabsf(measurement.x - previousMeasurement.x) > mmThreshold) ||
		(fabsf(measurement.y - previousMeasurement.y) > mmThreshold) )
		{
			/*
			printTEL++;
			if (printTEL > 100)
			{
				vPrintString("xOffset is:%.3f, yOffset is:%.3f.\n", measurement.x, measurement.y);
				printTEL = 0;
			}*/
			
			if (handle_OffsetQueue != NULL)
			{
				if (xQueueOverwrite(handle_OffsetQueue, &measurement) == pdPASS)
				{
					previousMeasurement = measurement;
				}
			}
		}
	}//end-Offset bepaling	
}



///////////////////////////////////////////////////////////////////////////////
// void InputHandlerTask(void *pvParameters)
/*
 * Bewaakt Start, Stop en Reset en verwerkt periodiek analoge ingangen.
 * Invoer: pvParameters wordt niet gebruikt.
 * Uitvoer: geen returnwaarde; zet eventbits en queue-waarden voor andere taken.
 */
void InputHandlerTask(void *pvParameters)
{
	vPrintString("> starting InputHandlerTask\n");
	
	bool stopWasPressed = false;
	bool teachEventSent = false;
	uint16_t stopPressTicks = 0U;

	//adc_EnableChannel(stroomChannel);
	adc_EnableChannel(xDisturbanceChannel);
	adc_EnableChannel(yDisturbanceChannel);
	uint16_t xAdcRaw = 0, yAdcRaw = 0;
	
	// Apart aangeven dat deze task actief is
	vPrintString("> running InputHandlerTask\n");
	xEventGroupSetBits(handle_ThreadEventGroup, BIT_0);
	
	while (true)
	{		
		// STOP
		//ProcessStopButton();
		//als stopknop wordt ingedrukt
		if (IsButtonPressed(PCB_SWITCH_STOP, BIT_STOP))
		{
			if (!stopWasPressed)
			{
				stopWasPressed = true;
				teachEventSent = false;
				stopPressTicks = 0U;
			}

			if (stopPressTicks < STOP_LONG_PRESS_TICKS)
			{
				stopPressTicks++;
			}

			// Indien knop langer dan 5 sec vastgehouden wordt.
			if ((stopPressTicks >= STOP_LONG_PRESS_TICKS) && !teachEventSent)
			{
				vPrintString("> STOP lang ingedrukt: TEACH aangevraagd.\n");
				xEventGroupSetBits(handle_ButtonEventGroup, EVT_TEACH_BUTTON);
				teachEventSent = true;
			}
		}
		// Als stopknop losgelaten wordt.
		else if (stopWasPressed)
		{
			if (!teachEventSent && (stopPressTicks >= STOP_DEBOUNCE_TICKS))
			{
				vPrintString("> STOP button is pressed!\n");
				xEventGroupSetBits(handle_ButtonEventGroup, EVT_STOP_BUTTON);
			}

			stopWasPressed = false;
			teachEventSent = false;
			stopPressTicks = 0U;
		}
		
		// START wordt losgelaten
		if ( ButtonWasPressed(PCB_SWITCH_START, BIT_START) )
		{
			vPrintString("> START button is pressed!\n");
			xEventGroupSetBits(handle_ButtonEventGroup, EVT_START_BUTTON);
		}		
		// RESET
		if ( ButtonWasPressed(PCB_SWITCH_RESET, BIT_RESET) )
		{
			vPrintString("> RESET button is pressed!\n");
			xEventGroupSetBits(handle_ButtonEventGroup, EVT_RESET_BUTTON);
		}

		//if (TCP_COMP == 1)
		//{
		// Analoge kanalen uitlezen
			adc_StartConversion();
			while ( (adc_IsConversionReady(xDisturbanceChannel) == false) || (adc_IsConversionReady(yDisturbanceChannel) == false)) //(adc_IsConversionReady(stroomChannel) == false)
			{
				taskSleep(0);
			}

			//ProcessCurrentSensorData( adc_ReadData(stroomChannel));

			xAdcRaw = (uint16_t)adc_ReadData(xDisturbanceChannel);
			yAdcRaw = (uint16_t)adc_ReadData(yDisturbanceChannel);
		
			ProcessOffsetPositionData(xAdcRaw, yAdcRaw);
		//}
		taskSleep(1);
	}
/*Should never get Here*/
}
