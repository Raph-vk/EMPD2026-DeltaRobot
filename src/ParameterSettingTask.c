/*
 * ParameterSettingTask.c
 *
 * Created: 23-11-2023 13:20:17
 *  Author: rasmsmee
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// library & HAL includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"
#include "ADCLib.h"
#include "SwitchLib.h"
#include "map.h"

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "ApplicationTasks.h"
#include "ParameterSettingTask.h"
#include "bits.h"

///////////////////////////////////////////////////////////////////////////////
// void ParameterSettingTask(void *pvParameters)
//
// Deze Taak leest de ADC-channel, afkomstig uit de potmeter, vertaald dit
// naar een represenatieve bandbreedte waarde en plaatst deze in queue.
//
// Wanneer een waarde is uitgelezen wordt event vrij gegeven om controlTask te starten.

void ParameterSettingTask(void *pvParameters)
{
	uint32_t adcData = 0;
	uint8_t  adcChannel = 0;
	uint8_t  buttonNumber = 3;
	
	double   currentWblFactor  = -1.0;	// start with invalid value
	double   previousWblFactor =  0.0;
	
	vPrintString("> starting ParameterSettingTask\n");

	// init channel
	adc_EnableChannel(adcChannel);
		
	while(true)
	{
		// start analog naar digitale conversie en wacht tot klaar is.
		adc_StartConversion();
		while(adc_IsConversionReady(adcChannel) == false)
		{
			taskSleep(0);
		}

		// Lees data waarde uit en normaliseer dit naar een bandbreedte factor.
		adcData = adc_ReadData(adcChannel);
		currentWblFactor = fmap(adcData, ADC_MIN_VALUE, ADC_MAX_VALUE, WBLFACTOR_MIN, WBLFACTOR_MAX);
		
		// Indien verschil tussen actuele en vorige waarde groot genoeg is,
		// wordt deze geplaatst in Queue.
		if ( fabs(currentWblFactor - previousWblFactor) > WBLTHRESHOLD ) //fabs() maakt absolute waarde.
		{
			xQueueOverwrite(handle_ParameterQueue, &currentWblFactor);
			vPrintString("> wblFactor set to %.3f\n", currentWblFactor);
			previousWblFactor = currentWblFactor;
		}
		
		// pressing SW4 (buttonNumber == 3) shows the current value of the WblFactor, 
		// it does not update the queue.
		// Show PREVIOUS value (previousWblFactor), as it might be not updated (yet)!!
		
		if (switch_IsPressed(buttonNumber))
		{
			vPrintString("> current wblFactor = %.3f\n", previousWblFactor); 
			// wait until button released:
			while (switch_IsPressed(buttonNumber))
			{
			}
		}

		// after first pass: let control thread know that valid 
		// data is available for use:
		xEventGroupSetBits( handle_ThreadEventGroup, BIT_1 ); 
	
		taskSleep(10);
	}
}
