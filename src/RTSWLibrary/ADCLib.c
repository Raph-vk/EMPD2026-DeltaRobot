/*
 * ADCLib.c
 *
 * Created: 2-10-2022 12:04:27
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "ADCLib.h"

///////////////////////////////////////////////////////////////////////////////
// file global data

static uint32_t G_EOCBits[] =
{
	ADC_CHANNEL_7_EOC,	//EOC bit for Due channel 0 = SAM channel 7
	ADC_CHANNEL_6_EOC,  // Due ADC channel 1
	ADC_CHANNEL_5_EOC,	// Due ADC channel 2
	ADC_CHANNEL_4_EOC,	// Due ADC channel 3
	ADC_CHANNEL_3_EOC,	// Due ADC channel 4
	ADC_CHANNEL_2_EOC,	// Due ADC channel 5
	ADC_CHANNEL_1_EOC,	// Due ADC channel 6
	ADC_CHANNEL_0_EOC,	// Due ADC channel 7
	
	ADC_CHANNEL_10_EOC,	// Due ADC channel 8
	ADC_CHANNEL_11_EOC,	// Due ADC channel 9
	ADC_CHANNEL_12_EOC,	// Due ADC channel 10
	ADC_CHANNEL_13_EOC,	// Due ADC channel 11
};

///////////////////////////////////////////////////////////////////////////////
// uint8_t adc_MapChanneltoSAM(uint8_t dueChannelNumber)

uint8_t adc_MapChanneltoSAM(uint8_t dueChannelNumber)
{
	int8_t samChannel = -1;
	
	if ( adc_IsValidChannel(dueChannelNumber) )
	{
		samChannel = ADC_CHANNEL_7 - dueChannelNumber;
	}
	
	return samChannel;
}

///////////////////////////////////////////////////////////////////////////////
// void adc_EnableChannel(uint8_t channel)

void adc_EnableChannel(uint8_t channel)
{
	if ( adc_IsValidChannel(channel) )
	{
		adc_enable_channel(ADC, ADC_CHANNEL_7 - channel);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void adc_DisableChannel(uint8_t channel)

void adc_DisableChannel(uint8_t channel)
{
	if ( adc_IsValidChannel(channel) )
	{
		adc_disable_channel(ADC, ADC_CHANNEL_7 - channel);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void adc_StartConversion(void)

void adc_StartConversion(void)
{
	adc_start(ADC);	
}


///////////////////////////////////////////////////////////////////////////////
// bool adc_IsConversionReady(uint8_t channel)

bool adc_IsConversionReady(uint8_t channel)
{
	bool isReady = false;
	uint32_t eocbit = 0;
	
	if ( adc_IsValidChannel(channel) )
	{
		eocbit = G_EOCBits[channel];
	
		if ((adc_get_status(ADC) & eocbit) == 0)
		{
			isReady = false;
		}
		else
		{
			isReady = true;
		}
	}

	return isReady;
}

///////////////////////////////////////////////////////////////////////////////
// uint32_t adc_ReadData(uint8_t channel)

uint32_t adc_ReadData(uint8_t channel)
{
	uint32_t adcValue = 0;
	
	if ( adc_IsValidChannel(channel) )
	{
		adcValue = adc_get_channel_value(ADC, ADC_CHANNEL_7 - channel);
	}
	
	return adcValue;
}

//////////////////////////////////////////////////////////////////////////////
// bool adc_IsValidChannel(uint8_t channel)

bool adc_IsValidChannel(uint8_t channel)
{
	bool isValid = false;
	
	if (channel <= ADC_MAX_CHANNEL)	
	{
		isValid = true;
	}
	
	return isValid;
}

//////////////////////////////////////////////////////////////////////////////
// void adc_Init(void)

void adc_Init(void)
{
	pmc_enable_periph_clk(ID_ADC);
	
	adc_init(ADC, sysclk_get_main_hz(), ADC_CLOCK, 8);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_12);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
}


//////////////////////////////////////////////////////////////////////////////
// float adc_Fmap(float x, float in_min, float in_max, float out_min, float out_max)

float adc_Fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//////////////////////////////////////////////////////////////////////////////
// float adc_ValueToVoltage(uint16_t adcValue)

float adc_ValueToVoltage(uint16_t adcValue)
{
	float voltage = 0.0;
	
	voltage = adc_Fmap(adcValue, 0, 4095, 0.0, 3.3);
	
	return voltage;
}
