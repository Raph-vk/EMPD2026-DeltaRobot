/*
 * DisturbanceCompensation.c
 *
 * Created: 13-06-2026
 * Author: Robbe van Eekelen
 *
 * Meet de X- en Y-verstoring van het frame met twee analoge sensoren.
 * Bij initialisatie wordt de nulpositie bepaald. Daarna kan de updatefunctie
 * door een losse taak worden aangeroepen om de nieuwste verstoring in mm naar
 * de queue te schrijven.
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
// RTSW shield analog input channel 3 - X disturbance
// RTSW shield analog input channel 4 - Y disturbance
uint8_t xDisturbanceChannel = 3;
uint8_t yDisturbanceChannel = 4;

///////////////////////////////////////////////////////////////////////////////
// Instellingen omrekening
static const float adcMaxValue = 4095.0f;
static const float disturbanceStrokeMm = 30.0f;
static const uint16_t calibrationSamples = 1000U;
static const uint32_t disturbanceSampleTimeMs = 20U;

// Aantal decimalen waarmee de mm-waarde wordt doorgestuurd.
static const uint8_t disturbanceDecimals = 2U;
#define DISTURBANCE_AVERAGE_SAMPLES    (12U)

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

static uint16_t xAdcBuffer[DISTURBANCE_AVERAGE_SAMPLES] = {0U};
static uint16_t yAdcBuffer[DISTURBANCE_AVERAGE_SAMPLES] = {0U};
static uint32_t xAdcSum = 0U;
static uint32_t yAdcSum = 0U;
static uint8_t adcFilterIndex = 0U;
static uint8_t adcFilterReady = 0U;

///////////////////////////////////////////////////////////////////////////////
// static float roundToDecimals(float value, uint8_t decimals)
/*
 * Rondt een floatwaarde af op het opgegeven aantal decimalen.
 */
static float roundToDecimals(float value, uint8_t decimals)
{
	float factor = 1.0f;

	for (uint8_t i = 0U; i < decimals; i++)
	{
		factor *= 10.0f;
	}

	if (value >= 0.0f)
	{
		return (float)((int32_t)(value * factor + 0.5f)) / factor;
	}
	else
	{
		return (float)((int32_t)(value * factor - 0.5f)) / factor;
	}
}

///////////////////////////////////////////////////////////////////////////////
// static void UpdateMovingAverageFilter(...)
/*
 * Werkt het moving average filter bij met de nieuwste X- en Y-ADC-meting.
 */
static void UpdateMovingAverageFilter(uint16_t xAdcRaw,
                                      uint16_t yAdcRaw,
                                      float *xAdcFiltered,
                                      float *yAdcFiltered)
{
	if (adcFilterReady == 0U)
	{
		xAdcSum = 0U;
		yAdcSum = 0U;

		for (uint8_t i = 0U; i < DISTURBANCE_AVERAGE_SAMPLES; i++)
		{
			xAdcBuffer[i] = xAdcRaw;
			yAdcBuffer[i] = yAdcRaw;
			xAdcSum += xAdcRaw;
			yAdcSum += yAdcRaw;
		}

		adcFilterIndex = 0U;
		adcFilterReady = 1U;
	}
	else
	{
		xAdcSum -= xAdcBuffer[adcFilterIndex];
		yAdcSum -= yAdcBuffer[adcFilterIndex];
		xAdcBuffer[adcFilterIndex] = xAdcRaw;
		yAdcBuffer[adcFilterIndex] = yAdcRaw;
		xAdcSum += xAdcRaw;
		yAdcSum += yAdcRaw;

		adcFilterIndex++;
		if (adcFilterIndex >= DISTURBANCE_AVERAGE_SAMPLES)
		{
			adcFilterIndex = 0U;
		}
	}

	*xAdcFiltered = (float)xAdcSum / (float)DISTURBANCE_AVERAGE_SAMPLES;
	*yAdcFiltered = (float)yAdcSum / (float)DISTURBANCE_AVERAGE_SAMPLES;
}

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
	xAdcSum = 0U;
	yAdcSum = 0U;
	adcFilterIndex = 0U;
	adcFilterReady = 0U;

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
 * queue. Deze functie is bedoeld om periodiek aangeroepen te worden nadat
 * DisturbanceCompensation_Init() klaar is.
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

	float xAdcFiltered = 0.0f;
	float yAdcFiltered = 0.0f;

	UpdateMovingAverageFilter(xAdcRaw, yAdcRaw, &xAdcFiltered, &yAdcFiltered);

	DisturbanceMeasurement_t measurement;

	measurement.x_disturbance_mm =
		((xAdcFiltered - xZeroAdc) / adcMaxValue) * disturbanceStrokeMm;

	measurement.y_disturbance_mm =
		((yAdcFiltered - yZeroAdc) / adcMaxValue) * disturbanceStrokeMm;

	measurement.x_disturbance_mm =
		roundToDecimals(measurement.x_disturbance_mm, disturbanceDecimals);

	measurement.y_disturbance_mm =
		roundToDecimals(measurement.y_disturbance_mm, disturbanceDecimals);
		
		
	vPrintString("> disturbance X=%.2f mm, Y=%.2f mm\n",
	measurement.x_disturbance_mm,
	measurement.y_disturbance_mm);
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
