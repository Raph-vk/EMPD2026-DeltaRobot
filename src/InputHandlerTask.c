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
//#define ADC_REFERENCE_VOLTAGE         (3.3f)
//#define CURRENT_SENSOR_VOLTS_PER_AMP  (0.1f)

#define adcMaxValue					  (4095.0f)
#define mmStroke					(30.43f)
#define mmThreshold					(0.01f)

#define stroomChannel (2U)
#define xDisturbanceChannel (3U)
#define yDisturbanceChannel (4U)

#define calibrationSamples (100U)
#define MOVING_AVERAGE_SAMPLES (12U)

///////////////////////////////////////////////////////////////////////////////
//static bool hasCurrentSample = false;
//static uint32_t vorigeStroomData = 0;
//static float zeroCurrentVoltage = 2.5f;        // Better: measure this during startup

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
// static void UpdateMovingAverageFilter(...)
/*
 * Werkt het moving average filter bij met de nieuwste X- en Y-ADC-meting.
 */
static void MovingAverageFilter(uint16_t xAdcRaw, uint16_t yAdcRaw, OffsetPos_t *ADCfiltered, bool resetFilter)
{
	static uint8_t ADCsampleIndex = 0;
	static uint16_t xADCbuffer[MOVING_AVERAGE_SAMPLES] = {0}, yADCbuffer[MOVING_AVERAGE_SAMPLES] = {0};	
	static uint32_t xADCsum = 0, yADCsum = 0;
	static bool ADCfilterReady = false;
	
	//als filter nog niet geinitialiseerd
	if (!ADCfilterReady || resetFilter)
	{
		//sommaties op nul zetten
		xADCsum = 0;
		yADCsum = 0;

		// iedere sample vullen met huidig gemeten waarde
		for (uint8_t i = 0; i < MOVING_AVERAGE_SAMPLES; i++)
		{
			//gehele buffer vullen met actuele meetwaarde
			xADCbuffer[i] = xAdcRaw;
			yADCbuffer[i] = yAdcRaw;

			//sommatie optellen
			xADCsum += xAdcRaw;
			yADCsum += yAdcRaw;
		}

		ADCsampleIndex = 0;
		ADCfilterReady = true;
	}
	// Volgens "Ringbuffer" 
	// de oudste waarde met nieuwe waarde vervangen 
	//en gemiddelde uitsturen
	else
	{
		//oude waarde van sommatie aftrekken
		xADCsum -= xADCbuffer[ADCsampleIndex];
		yADCsum -= yADCbuffer[ADCsampleIndex];
		
		//nieuwe waarde in buffer plaatsen
		xADCbuffer[ADCsampleIndex] = xAdcRaw;
		yADCbuffer[ADCsampleIndex] = yAdcRaw;
		
		//nieuwe waarde in sommatie plaatsen
		xADCsum += xAdcRaw;
		yADCsum += yAdcRaw;

		//sampleIndex optellen
		ADCsampleIndex++;
		if (ADCsampleIndex >= MOVING_AVERAGE_SAMPLES)
		{
			ADCsampleIndex = 0;
		}
	}

	ADCfiltered->x = (float)xADCsum / (float)MOVING_AVERAGE_SAMPLES;
	ADCfiltered->y = (float)yADCsum / (float)MOVING_AVERAGE_SAMPLES;
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
	static OffsetPos_t ADCfiltered = {0.0f, 0.0f};

	static uint32_t xSum = 0, ySum = 0;
	static uint8_t sampleCount = 0;

	static bool zeroingActive = true;
	static bool offsetGehomed = true;
	static bool resetFilter = true;

	
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
			

			//filter wordt eenmalig gereset 
			resetFilter = true;
			ADCfiltered = (OffsetPos_t){0.0f, 0.0f};
			MovingAverageFilter(xAdcRaw, yAdcRaw, &ADCfiltered, resetFilter);
			resetFilter = false;
			
			// nieuwe waarde naar queue schrijven
			measurement.x = ((ADCfiltered.x - ZeroPos.x) / adcMaxValue) * mmStroke;
			measurement.y = ((ADCfiltered.y - ZeroPos.y) / adcMaxValue) * mmStroke;
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
		//ADCfiltered = (OffsetPos_t){0.0f, 0.0f};
		//MovingAverageFilter(xAdcRaw, yAdcRaw, &ADCfiltered, resetFilter);
		//measurement.x = ((ADCfiltered.x - ZeroPos.x) / adcMaxValue) * mmStroke;
		//measurement.y = ((ADCfiltered.y - ZeroPos.y) / adcMaxValue) * mmStroke;
		
		measurement.x = ((xAdcRaw - ZeroPos.x) / adcMaxValue) * mmStroke;
		measurement.y = ((yAdcRaw - ZeroPos.y) / adcMaxValue) * mmStroke;
	
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

	//adc_EnableChannel(stroomChannel);
	adc_EnableChannel(xDisturbanceChannel);
	adc_EnableChannel(yDisturbanceChannel);
	uint16_t xAdcRaw = 0, yAdcRaw = 0;
	
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


		if (TCP_COMP == 1)
		{
			// Analoge kanalen uitlezen
			adc_StartConversion();
			while ( (adc_IsConversionReady(xDisturbanceChannel) == false) || (adc_IsConversionReady(yDisturbanceChannel) == false))
			{
				taskSleep(0);
			}
		
			//ProcessCurrentSensorData( adc_ReadData(stroomChannel));

			xAdcRaw = (uint16_t)adc_ReadData(xDisturbanceChannel);
			yAdcRaw = (uint16_t)adc_ReadData(yDisturbanceChannel);
		
			ProcessOffsetPositionData(xAdcRaw, yAdcRaw);
		}
		taskSleep(1);
	}
/*Should never get Here*/
}
