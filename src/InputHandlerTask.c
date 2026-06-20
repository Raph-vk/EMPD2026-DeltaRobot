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
#define ADC_REFERENCE_VOLTAGE         3.3f
#define CURRENT_SENSOR_VOLTS_PER_AMP  0.1f
#define adcMaxValue					  4095.0f
#define mmStroke					30.0f
#define mmThreshold					0.01f

#define xDisturbanceChannel 3U
#define yDisturbanceChannel 4U

#define calibrationSamples 100U
///////////////////////////////////////////////////////////////////////////////
//file globals

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
// static void ProcessPotmeterData(uint32_t potData)
//#define PERCENT_STEP_SIZE       5u
//static uint32_t lastProcent = UINT32_MAX;
/*
 * Zet de ADC-waarde van de potmeter om naar een percentage in stappen van 5%.
 * Invoer: potData is de ruwe ADC-waarde.
 * Uitvoer: geen returnwaarde; schrijft de nieuwe procentwaarde naar handle_potQueue.
 */
/*
static void ProcessPotmeterData(uint32_t potData)
{
    uint32_t procent;
    uint32_t adcPercentScale = potData * 100u;

    if (adcPercentScale < (ADC_MAX_VALUE * PERCENT_STEP_SIZE))
    {
        procent = 0u;
    }
    else if (adcPercentScale > (ADC_MAX_VALUE * (100u - PERCENT_STEP_SIZE) ))
    {
        procent = 100u;
    }
    else
    {
        uint32_t step = ((potData * 20) + (ADC_MAX_VALUE / 2u)) / ADC_MAX_VALUE;
        procent = step * PERCENT_STEP_SIZE;
    }

    if (procent != lastProcent)
    {
        xQueueOverwrite(handle_potQueue, &procent);
        vPrintString("> Operation Percentage is set to %lu%%\n", (unsigned long)procent);
        lastProcent = procent;
    }
}
*/



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
// static void UpdateMovingAverageFilter(...)
#define MovingAverageSamples    (9U)
#define OFFSET_FILTER_X         (0U)
#define OFFSET_FILTER_Y         (1U)
#define FILTERCHANNELS		    (2U)
/*
 * Werkt het moving average filter bij voor een enkel ADC-kanaal.
 */
static void MovingAverageFilter(uint8_t fc, uint16_t AdcRaw, float *AdcFiltered, bool resetFilter)
{
	static uint8_t adcFilterReady[FILTERCHANNELS] = { 0U, 0U };
	static uint8_t adcFilterIndex[FILTERCHANNELS] = { 0U, 0U };

	static uint32_t adcSum[FILTERCHANNELS] = { 0U, 0U };
	static uint16_t adcBuffer[FILTERCHANNELS][MovingAverageSamples];

	//veiligheid afscherming
	if ((fc >= FILTERCHANNELS) || (AdcFiltered == NULL))
	{
		return;
	}

	// controleren geheugen gevuld/gereset moet worden.
	if ((adcFilterReady[fc] == 0U) || resetFilter)
	{
		adcSum[fc] = 0U;

		//door alle geheugens loopen
		for (uint8_t i = 0U; i < MovingAverageSamples; i++)
		{
			adcBuffer[fc][i] = AdcRaw;
			adcSum[fc] += AdcRaw;
		}

		adcFilterIndex[fc] = 0U;
		adcFilterReady[fc] = 1U;
	}
	// volgens RingBuffer de samples vervangen.
	else
	{
		uint8_t index = adcFilterIndex[fc];

		adcSum[fc] -= adcBuffer[fc][index];
		adcBuffer[fc][index] = AdcRaw;
		adcSum[fc] += AdcRaw;

		//naar volgende sample gaan.
		index++;
		if (index >= MovingAverageSamples)
		{
			index = 0U;
		}
		adcFilterIndex[fc] = index;
	}

	//Output waarde
	*AdcFiltered = (float)adcSum[fc] / (float)MovingAverageSamples;
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
	float xZeroPos = 0.0f;
	float yZeroPos = 0.0f;
		
	//oneindiggrote waarde zodat eerste keer altijd groot verschil is.
	OffsetPos_t previousMeasurement = { INFINITY, INFINITY }; 
	OffsetPos_t measurement = { 0.0f, 0.0f };

	bool zeroingActive = false;
	bool offsetZeroed = false;
	bool resetFilter = false;
	//bool previousMeasurementValid = false;
	//bool offsetQueueReadyPrinted = false;
	uint32_t xSum = 0, ySum = 0;
	uint16_t xAdcRaw = 0, yAdcRaw = 0;
	uint16_t sampleCount = 0;

	adc_EnableChannel(xDisturbanceChannel);
	adc_EnableChannel(yDisturbanceChannel);
	
	// Apart aangeven dat deze task actief is
	vPrintString("> running InputHandlerTask\n");
	xEventGroupSetBits(handle_ThreadEventGroup, BIT_0);
	
	while (true)
	{		
		// START
		if ( ButtonWasPressed(PCB_SWITCH_START, BIT_START) )
		{
			vPrintString("> START button is pressed!\n");
			xEventGroupSetBits(handle_ButtonEventGroup, EVT_START_BUTTON);
		}
		// STOP
		if ( ButtonWasPressed(PCB_SWITCH_STOP, BIT_STOP) )
		{
			vPrintString("> STOP button is pressed!\n");
			xEventGroupSetBits(handle_ButtonEventGroup, EVT_STOP_BUTTON);
		}
		// RESET
		if ( ButtonWasPressed(PCB_SWITCH_RESET, BIT_RESET) )
		{
			vPrintString("> RESET button is pressed!\n");
			xEventGroupSetBits(handle_ButtonEventGroup, EVT_RESET_BUTTON);
		}



		// Analoge kanalen uitlezen
		adc_StartConversion();
		while ((adc_IsConversionReady(xDisturbanceChannel) == false) ||
		(adc_IsConversionReady(yDisturbanceChannel) == false))
		{
			taskSleep(0);
		}
		xAdcRaw = (uint16_t)adc_ReadData(xDisturbanceChannel);
		yAdcRaw = (uint16_t)adc_ReadData(yDisturbanceChannel);

		// Als zero wordt aangevraagd, deze pakken en "zeroingActive" flag op true zetten
		if (xSemaphoreTake(handle_OffsetZeroRequest, 0) == pdTRUE)
		{
			zeroingActive = true;
			offsetZeroed = false;
			sampleCount = 0;
			xSum = 0;
			ySum = 0;
			xSemaphoreTake(handle_OffsetZeroDone, 0);
			vPrintString("> Offset zeroing requested.\n");
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
				// Als de hoeveelheid samples gehaald zijn.
				xZeroPos = (float)xSum / (float)calibrationSamples;
				yZeroPos = (float)ySum / (float)calibrationSamples;
				vPrintString("> Offset calibratie klaar.\n");
				
				//vPrintString("> xZeroAdc: %.2f\n", xZeroPos);
				//vPrintString("> yZeroAdc: %.2f\n", yZeroPos);
				zeroingActive = false;
				offsetZeroed = true;
				previousMeasurement.xOffset = INFINITY;
				previousMeasurement.yOffset = INFINITY;
				resetFilter = true;
				xSemaphoreGive(handle_OffsetZeroDone);
			}
		}
		else if (offsetZeroed)
		{
			float xAdcFiltered = 0.0f;
			float yAdcFiltered = 0.0f;

			MovingAverageFilter(OFFSET_FILTER_X, xAdcRaw, &xAdcFiltered, resetFilter);
			MovingAverageFilter(OFFSET_FILTER_Y, yAdcRaw, &yAdcFiltered, resetFilter);
			resetFilter = false;

			measurement.xOffset = ((xAdcFiltered - xZeroPos) / adcMaxValue) * mmStroke;
			measurement.yOffset = ((yAdcFiltered - yZeroPos) / adcMaxValue) * mmStroke;
				
			// Bepaal het verschil met de vorige meting die naar de queue is geschreven.
			// De treshold werkt dus niet ten opzichte van 0 mm, maar ten opzichte van
			// de laatst gebruikte verstoringswaarde.
			//
			// Als X of Y genoeg veranderd is, naar de queue schrijven.
			if ( (fabsf(measurement.xOffset - previousMeasurement.xOffset) > mmThreshold) ||
			     (fabsf(measurement.yOffset - previousMeasurement.yOffset) > mmThreshold) )
			{
				if (handle_OffsetQueue != NULL)
				{
					if (xQueueOverwrite(handle_OffsetQueue, &measurement) == pdPASS)
					{
						previousMeasurement = measurement;
					}
				}
			}
		}//end-Offset bepaling
		
		taskSleep(10);
	}
/*Should never get Here*/
}
