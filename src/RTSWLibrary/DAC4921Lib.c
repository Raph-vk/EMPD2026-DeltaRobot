/*
 * DAC4921Lib.c
 *
 * Created: 2-10-2022 15:39:42
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"
#include "SPILib.h"
#include "DAC4921Lib.h"
#include "Map.h"

///////////////////////////////////////////////////////////////////////////////
// file global data


///////////////////////////////////////////////////////////////////////////////
// table with Arduino Due pin assignment for DAC chip enables (active LOW)

static const uint8_t G_DACSelectPins[DAC_N_CHANNELS] =
{
	PIN_23,	// chip select DAC channel 0
	PIN_25,	// chip select DAC channel 1
	PIN_27,	// chip select DAC channel 2
	PIN_29,	// chip select DAC channel 3
};


///////////////////////////////////////////////////////////////////////////////
// void dac_Init(void)

void dac_Init(void)
{
	uint8_t channel = 0;
	
	spi_Init();	// DAC uses SPI, so initialize the SPI bus

	// initialize the CS* pins as outputs, initially inactive = HIGH:
	
	for (channel = 0; channel <= DAC_MAX_CHANNEL; channel++)
	{
		dio_SetPinDirection(G_DACSelectPins[channel], IOPORT_DIR_OUTPUT); 
		dio_SetPin(G_DACSelectPins[channel], true);
	}

		
	// initialize the DAC chips the first time by writing any value...

	for (channel = 0; channel <= DAC_MAX_CHANNEL; channel++)
	{
		dac_Write(channel, 0);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void dac_SelectChannel(uint8_t dacChannel)

void dac_SelectChannel(uint8_t dacChannel)
{
	uint8_t selectPin = 0;
	
	if (dacChannel <= DAC_MAX_CHANNEL)
	{
		selectPin = G_DACSelectPins[dacChannel];
		dio_SetPin(selectPin, false);	// select = set CE* LOW
	}
}

///////////////////////////////////////////////////////////////////////////////
// void dac_DeselectChannel(uint8_t dacChannel)

void dac_DeselectChannel(uint8_t dacChannel)
{
	uint8_t selectPin = 0;
	
	if (dacChannel <= DAC_MAX_CHANNEL)
	{
		selectPin = G_DACSelectPins[dacChannel];
		dio_SetPin(selectPin, true);	// deselect = set CE* HIGH
	}
}

///////////////////////////////////////////////////////////////////////////////
// void dac_Write(uint8_t dacChannel, uint16_t dacValue)

uint32_t dac_Write(uint8_t dacChannel, uint16_t dacValue)
{
	//uint8_t packet[2];
	status_code_t status = OPERATION_IN_PROGRESS;
	
	if (dacChannel <= DAC_MAX_CHANNEL)
	{
		// force 12 bit DAC data value & set control bits

		dacValue = ((dacValue & 0x0fff) | (DAC_SELECT_A | DAC_VREF_BUFFERED | DAC_GAINSELECT_1 | DAC_POWER_ON));

		//packet[0] = (dacValue & 0xff00) >> 8;	// high byte
		//packet[1] = (dacValue & 0x00ff);		// low byte
	
		dac_SelectChannel(dacChannel);
		//spi_WritePacket(packet, 2);
		status = spi_WriteWord(dacValue);
		dac_DeselectChannel(dacChannel);
	}
	
	return status;
}


///////////////////////////////////////////////////////////////////////////////
// void dac_SelectAllChannels(void)

void dac_SelectAllChannels(void)
{
	uint8_t channel = 0;
	
	for (channel = 0; channel <= DAC_MAX_CHANNEL; channel++)
	{
		dac_SelectChannel(channel);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void dac_DeselectChannel(uint8_t dacChannel)

void dac_DeselectAllChannels(void)
{
	uint8_t channel = 0;
	
	for (channel = 0; channel <= DAC_MAX_CHANNEL; channel++)
	{
		dac_DeselectChannel(channel);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void dac_WriteAll(uint16_t dacValue)
//
// write dacValue to all DAC's in parallel

void dac_WriteAll(uint16_t dacValue)
{
	// force 12 bit DAC data value & set control bits

	dacValue = ((dacValue & 0x0fff) | (DAC_SELECT_A | DAC_VREF_BUFFERED | DAC_GAINSELECT_1 | DAC_POWER_ON));

	dac_SelectAllChannels();
	spi_WriteWord(dacValue);
	dac_DeselectAllChannels();
}


///////////////////////////////////////////////////////////////////////////////
// void dac_SetOutputVoltage(uint8_t dacChannel, float outputVoltage)
//
// Vout = -10 + 8*Vdac
// Vdac = Vref * (dacValue / 2**12)
// Vref = 2.5 Volt
// ==> dacValue = (Vout + 10) * (2**12) / (8 * Vref)

void dac_SetOutputVoltage(uint8_t dacChannel, float outputVoltage)
{
	float dacValue = 0.0;
	
	outputVoltage = constrain(outputVoltage, DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE);
	
	dacValue = fmap(outputVoltage,	DAC_MIN_OUTPUTVOLTAGE, DAC_MAX_OUTPUTVOLTAGE, 
									DAC_MIN_VALUE, DAC_MAX_VALUE);	

	dac_Write(dacChannel, (uint16_t)dacValue);
}