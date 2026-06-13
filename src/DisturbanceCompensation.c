/*
 * DisturbanceCompensation.c
 *
 * Created: 13-06-2026
 * Author: Robbe van Eekelen
 *
 * Meet de X- en Y-verstoring van het frame met twee analoge sensoren.
 * Bij initialisatie wordt de nulpositie bepaald. Daarna kan de updatefunctie
 * in de 1 ms regel-lus worden aangeroepen om de nieuwste verstoring in mm
 * naar de queue te schrijven.
 */

#include "DisturbanceCompensation.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "ADCLib.h"
#include "TaskSleep.h"
#include "vPrintString.h"

///////////////////////////////////////////////////////////////////////////////
// Analoge kanalen
// ch4 - X disturbance
// ch5 - Y disturbance
uint8_t xDisturbanceChannel = 4;
uint8_t yDisturbanceChannel = 5;

///////////////////////////////////////////////////////////////////////////////
// Instellingen omrekening
static const float adcMaxValue = 4095.0f;
static const float disturbanceStrokeMm = 30.0f;
static const uint16_t calibrationSamples = 1000U;


/*
 * Tresholdwaarde voor schrijven naar de queue.
 * Een nieuwe meting wordt pas doorgestuurd als deze minimaal zoveel mm afwijkt
 * van de vorige doorgestuurde meting. Dit filtert ADC-ruis en kleine trillingen
 * weg, zonder dat de regeling blijft hangen op een oude verstoringswaarde.
 */
static const float disturbanceTresholdMm = 0.02f;
///////////////////////////////////////////////////////////////////////////////
// file globals
static QueueHandle_t disturbanceQueue = NULL;

static float xZeroAdc = 0.0f;
static float yZeroAdc = 0.0f;

// Laatste meting die daadwerkelijk naar de queue is geschreven.
static DisturbanceMeasurement_t previousMeasurement = {0.0f, 0.0f};

// Voorkomt dat de allereerste meting na initialisatie wordt weggefilterd.
static uint8_t previousMeasurementValid = 0U;

///////////////////////////////////////////////////////////////////////////////
// void DisturbanceCompensation_Init(QueueHandle_t queue)
/*
 * Initialiseert de verstoringsmeting.
 * De ADC-kanalen worden aangezet en de huidige sensorstanden worden opgeslagen
 * als nulpositie.
 *
 * Invoer: queue is de queue waar DisturbanceMeasurement_t metingen in komen.
 * Uitvoer: geen returnwaarde.
 */
void DisturbanceCompensation_Init(QueueHandle_t queue)
{
	uint32_t xSum = 0U;
	uint32_t ySum = 0U;

	disturbanceQueue = queue;
	previousMeasurementValid = 0U;

	adc_EnableChannel(xDisturbanceChannel);
	adc_EnableChannel(yDisturbanceChannel);

	vPrintString("> Disturbance calibratie start\n");
	vPrintString("> Zet X/Y meting in nulpositie\n");

	taskSleep(1000);

	for (uint16_t i = 0U; i < calibrationSamples; i++)
	{
		adc_StartConversion();

		while ((adc_IsConversionReady(xDisturbanceChannel) == false) ||
		       (adc_IsConversionReady(yDisturbanceChannel) == false))
		{
			taskSleep(0);
		}

		xSum += adc_ReadData(xDisturbanceChannel);
		ySum += adc_ReadData(yDisturbanceChannel);

		taskSleep(1);
	}

	xZeroAdc = (float)xSum / (float)calibrationSamples;
	yZeroAdc = (float)ySum / (float)calibrationSamples;

	vPrintString("> Disturbance calibratie klaar\n");
	vPrintString("> xZeroAdc: %ld\n", (long)xZeroAdc);
	vPrintString("> yZeroAdc: %ld\n", (long)yZeroAdc);
}

///////////////////////////////////////////////////////////////////////////////
// void DisturbanceCompensation_UpdateQueue(void)
/*
 * Meet de actuele X- en Y-verstoring en schrijft de nieuwste meting naar de
 * queue. Deze functie is bedoeld om mee te lopen in de harde 1 ms regel-lus.
 *
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; schrijft DisturbanceMeasurement_t naar de queue.
 */
void DisturbanceCompensation_UpdateQueue(void)
{
	if (disturbanceQueue == NULL)
	{
		return;
	}

	adc_StartConversion();

	while ((adc_IsConversionReady(xDisturbanceChannel) == false) ||
	       (adc_IsConversionReady(yDisturbanceChannel) == false))
	{
		taskSleep(0);
	}

	uint16_t xAdcRaw = (uint16_t)adc_ReadData(xDisturbanceChannel);
	uint16_t yAdcRaw = (uint16_t)adc_ReadData(yDisturbanceChannel);

	DisturbanceMeasurement_t measurement;

	measurement.x_disturbance_mm =
	(((float)xAdcRaw - xZeroAdc) / adcMaxValue) * disturbanceStrokeMm;

	measurement.y_disturbance_mm =
	(((float)yAdcRaw - yZeroAdc) / adcMaxValue) * disturbanceStrokeMm;

	/*
	 * Bepaal het verschil met de vorige meting die naar de queue is geschreven.
	 * De treshold werkt dus niet ten opzichte van 0 mm, maar ten opzichte van
	 * de laatst gebruikte verstoringswaarde.
	 */
	float xDeltaMm = measurement.x_disturbance_mm - previousMeasurement.x_disturbance_mm;
	float yDeltaMm = measurement.y_disturbance_mm - previousMeasurement.y_disturbance_mm;

	/*
	 * Als zowel X als Y minder dan disturbanceTresholdMm zijn veranderd,
	 * wordt de meting gezien als ruis en niet opnieuw naar de queue geschreven.
	 */
	if ((previousMeasurementValid != 0U) &&
	    (xDeltaMm > -disturbanceTresholdMm) &&
	    (xDeltaMm < disturbanceTresholdMm) &&
	    (yDeltaMm > -disturbanceTresholdMm) &&
	    (yDeltaMm < disturbanceTresholdMm))
	{
		return;
	}

	previousMeasurement = measurement;
	previousMeasurementValid = 1U;
	xQueueOverwrite(disturbanceQueue, &measurement);
}
