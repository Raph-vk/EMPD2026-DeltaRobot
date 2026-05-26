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
// potmeter percentage instellingen
#define PERCENT_STEP_SIZE       5u
static uint32_t lastProcent = UINT32_MAX;

///////////////////////////////////////////////////////////////////////////////
// static void ProcessPotmeterData(uint32_t potData)
/*
 * Zet de ADC-waarde van de potmeter om naar een percentage in stappen van 5%.
 * Invoer: potData is de ruwe ADC-waarde.
 * Uitvoer: geen returnwaarde; schrijft de nieuwe procentwaarde naar handle_potQueue.
 */
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

///////////////////////////////////////////////////////////////////////////////
// status stroomsensor
static bool hasCurrentSample = false;
static uint32_t vorigeStroomData = 0;
static float zeroCurrentVoltage = 2.5f;        // Better: measure this during startup

///////////////////////////////////////////////////////////////////////////////
// static void ProcessCurrentSensorData(uint32_t stroomData)
/*
 * Zet de ADC-waarde van de stroomsensor om naar een stroomwaarde in ampere.
 * Invoer: stroomData is de ruwe ADC-waarde van de sensor.
 * Uitvoer: geen returnwaarde; schrijft de berekende stroom naar handle_stroomQueue.
 */
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
}



///////////////////////////////////////////////////////////////////////////////
uint8_t stroomChannel = 3;
uint8_t potChannel = 4;

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

	adc_EnableChannel(stroomChannel);
	//adc_EnableChannel(potChannel);

	uint8_t i = 0;
	
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
		
		if (i >= 10)
		{
			i = 0;
			
			//Starts conversie van alle kanalen en wacht tot klaar zijn.
			adc_StartConversion();
			while (
				(adc_IsConversionReady(stroomChannel) == false)) //(adc_IsConversionReady(potChannel) == false) ||
			{
				taskSleep(0);
			}

			ProcessCurrentSensorData( adc_ReadData(stroomChannel));
			//ProcessPotmeterData( adc_ReadData(potChannel));
		}
		else
		{
			i++;
		}
		taskSleep(10);
	}
/*Should never get Here*/
}
