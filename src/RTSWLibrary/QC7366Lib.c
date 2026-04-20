/*
 * QC7366Lib.c
 *
 * Created: 14-10-2022 15:51:36
 *  Author: Roel Smeets
 
 * RASM, changed 16-04-2026:
 * added PIN_TX3 and PIN_RX3 as select signals for use with external SPI bus
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"
#include "SPILib.h"
#include "QC7366Lib.h"


///////////////////////////////////////////////////////////////////////////////
// table with Arduino Due pin assignment for LS7366 chip select enables 
// (active LOW)

static const uint8_t G_QCSelectPins[QC_N_CHANNELS] =
{
	PIN_47,		// QUADSEL0*, onboard counter channel 0
	PIN_49,		// QUADSEL1*, onboard counter channel 1
	PIN_TX3,	// SPISEL6*, external counter channel 2
	PIN_RX3,	// SPISEL7*, external counter channel 3
};

///////////////////////////////////////////////////////////////////////////////
// void qc_Init(void)

void qc_Init(void)
{
	uint8_t channel		 = 0;
	uint8_t defaultMode  = 0;
	mode_register_t modeRegister = QC_MODE_REGISTER_0;
	
	// mode depends on quadrature pulse definitions of the encoder used!!
	defaultMode = MODE_QC_1 | MODE_FREERUNNING | INDEX_RESETCNTR | INDEX_ASYNC | FILTERCLOCK_DIV_2;
	
	spi_Init();

	// initialize the QUADSEL(0/1)* pins as outputs, initially inactive = HIGH:
	
	for (channel = 0; channel <= QC_MAX_CHANNEL; channel++)
	{
		dio_SetPinDirection(G_QCSelectPins[channel], IOPORT_DIR_OUTPUT);
		dio_SetPin(G_QCSelectPins[channel], true);
	}

	for (channel = 0; channel <= QC_MAX_CHANNEL; channel++)
	{
		qc_WriteModeRegister(channel, modeRegister, defaultMode);
		qc_DisableCounter(channel);
		qc_ClearCountRegister(channel);
	}

}

///////////////////////////////////////////////////////////////////////////////
// void qc_SelectChannel(uint8_t qcChannel)

void qc_SelectChannel(uint8_t qcChannel)
{
	uint8_t selectPin = 0;
	
	if (qcChannel <= QC_MAX_CHANNEL)
	{
		selectPin = G_QCSelectPins[qcChannel];
		dio_SetPin(selectPin, false);	// select = set CE* LOW
	}
}

///////////////////////////////////////////////////////////////////////////////
// void qc_DeselectChannel(uint8_t qcChannel)

void qc_DeselectChannel(uint8_t qcChannel)
{
	uint8_t selectPin = 0;
	
	if (qcChannel <= QC_MAX_CHANNEL)
	{
		selectPin = G_QCSelectPins[qcChannel];
		dio_SetPin(selectPin, true);	// deselect = set CE* HIGH
	}
}

///////////////////////////////////////////////////////////////////////////////
// void qc_ClearStatusRegister(uint8_t channel)

void qc_ClearStatusRegister(uint8_t channel)
{
	qc_SendCommand(channel, CLR_STR);
}

///////////////////////////////////////////////////////////////////////////////
// uint8_t qc_ReadStatusRegister(uint8_t channel)

uint8_t qc_ReadStatusRegister(uint8_t channel)
{
	uint8_t statusValue = 0;
	
	if (channel <= QC_MAX_CHANNEL)
	{
		qc_SelectChannel(channel);
		spi_WriteByte(READ_STR);
		spi_ReadByte(&statusValue);
		qc_DeselectChannel(channel);
	}
	
	return statusValue;
}


///////////////////////////////////////////////////////////////////////////////
// bool qc_IsIndexSet(uint8_t channel)

bool qc_IsIndexSet(uint8_t channel)
{
	bool indexSet = false;
	uint8_t statusValue = 0;
	
	statusValue = qc_ReadStatusRegister(channel);
	
	if ((statusValue & IDX_BIT) != 0)
	{
		indexSet = true;
	}
	
	return indexSet;
	
}

///////////////////////////////////////////////////////////////////////////////
// void qc_WriteModeRegister(uint8_t channel, mode_register_t modeRegister, 
//							 uint8_t valueMDR)

void qc_WriteModeRegister(uint8_t channel, mode_register_t modeRegister, uint8_t valueMDR)
{
	uint8_t writeMDRCommand = 0;
	
	if ((channel <= QC_MAX_CHANNEL) && (modeRegister <= QC_MODE_REGISTER_1))
	{
		writeMDRCommand = (modeRegister == QC_MODE_REGISTER_0) ? WRITE_MDR0 : WRITE_MDR1;
		
		qc_SelectChannel(channel);
		spi_WriteByte(writeMDRCommand);
		spi_WriteByte(valueMDR);
		qc_DeselectChannel(channel);
	}
}

///////////////////////////////////////////////////////////////////////////////
// uint8_t qc_ReadModeRegister(uint8_t channel, mode_register_t modeRegister)

uint8_t qc_ReadModeRegister(uint8_t channel, mode_register_t modeRegister)
{
	uint8_t readMDRCommand = 0;
	uint8_t mdrValue = 0xff;
	
	if ((channel <= QC_MAX_CHANNEL) && (modeRegister <= QC_MODE_REGISTER_1))
	{
		readMDRCommand = (modeRegister == QC_MODE_REGISTER_0) ? READ_MDR0 : READ_MDR1;
		
		qc_SelectChannel(channel);
		spi_WriteByte(readMDRCommand);
		spi_ReadByte(&mdrValue);
		qc_DeselectChannel(channel);
	}
	
	return mdrValue;
}


///////////////////////////////////////////////////////////////////////////////
// void qc_ClearModeRegister(uint8_t channel, mode_register_t modeRegister)

void qc_ClearModeRegister(uint8_t channel, mode_register_t modeRegister)
{
	uint8_t readMDRCommand = 0;
	
	if ((channel <= QC_MAX_CHANNEL) && (modeRegister <= QC_MODE_REGISTER_1))
	{
		readMDRCommand = (modeRegister == QC_MODE_REGISTER_0) ? CLR_MDR0 : CLR_MDR1;
		qc_SendCommand(channel, readMDRCommand);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void qc_ClearCountRegister(uint8_t channel)

void qc_ClearCountRegister(uint8_t channel)
{
	qc_SendCommand(channel, CLR_CNTR);
}

///////////////////////////////////////////////////////////////////////////////
// int32_t qc_ReadCountRegister(uint8_t channel)

int32_t qc_ReadCountRegister(uint8_t channel)
{
	int32_t count = 0;
	uint8_t ix	  = 0;
	uint8_t val	  = 0;
	
	if (channel <= QC_MAX_CHANNEL)
	{
		qc_SelectChannel(channel);
		spi_WriteByte(READ_CNTR);
		
		for (ix = 0; ix < 4; ix++)
		{
			spi_ReadByte(&val);
			count = (count << 8) | val;
		}
		qc_DeselectChannel(channel);
	}
	
	return count;
}

///////////////////////////////////////////////////////////////////////////////
// void qc_TransferDataRegisterToCountRegister(uint8_t channel)

void qc_TransferDataRegisterToCountRegister(uint8_t channel)
{
	qc_SendCommand(channel, LOAD_CNTR);
}

///////////////////////////////////////////////////////////////////////////////
// void qc_WriteDataRegister(uint8_t channel, int32_t dtrValue)

void qc_WriteDataRegister(uint8_t channel, int32_t dtrValue)
{
	uint8_t ix = 0;
	uint8_t spiData = 0;
	
	if (channel <= QC_MAX_CHANNEL)
	{
		qc_SelectChannel(channel);
		spi_WriteByte(WRITE_DTR);
		
		for (ix = 0; ix < 4; ix++) // Most Significant byte first!
		{
			spiData = (uint8_t)(dtrValue >> 8*(3 - ix));	// shift right 24, 16, 8, 0
			spi_WriteByte(spiData);
		}		
		qc_DeselectChannel(channel);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void qc_TranferCountRegisterToOutputRegister(uint8_t channel)

void qc_TranferCountRegisterToOutputRegister(uint8_t channel)
{
	qc_SendCommand(channel, LOAD_OTR);
}

///////////////////////////////////////////////////////////////////////////////
// int32_t qc_ReadOutputRegister(uint8_t channel)

int32_t qc_ReadOutputRegister(uint8_t channel)
{
	int32_t count = 0;
	uint8_t ix = 0;
	uint8_t val = 0;
	
	if (channel <= QC_MAX_CHANNEL)
	{
		qc_SelectChannel(channel);
		spi_WriteByte(READ_OTR);
		
		for (ix = 0; ix < 4; ix++)
		{
			spi_ReadByte(&val);
			count = (count << 8) | val;
		}
		qc_DeselectChannel(channel);
	}
	
	return count;
}

///////////////////////////////////////////////////////////////////////////////
// void qc_DisableCounter(uint8_t channel)

void qc_DisableCounter(uint8_t channel)
{
	uint8_t mdrValue = 0;
	
	mdrValue  = qc_ReadModeRegister(channel, 1);
	mdrValue |= CNT_DISABLE;
	qc_WriteModeRegister(channel, 1, mdrValue);
}

///////////////////////////////////////////////////////////////////////////////
// void qc_DisableCounter(uint8_t channel)

void qc_EnableCounter(uint8_t channel)
{
	uint8_t mdrValue = 0;
	uint8_t modeRegister = 1;
	
	mdrValue  = qc_ReadModeRegister(channel, modeRegister);
	mdrValue &= ~CNT_DISABLE;
	qc_WriteModeRegister(channel, modeRegister, mdrValue);
}

///////////////////////////////////////////////////////////////////////////////
// void qc_SendCommand(uint8_t channel)

void qc_SendCommand(uint8_t channel, uint8_t commandByte)
{
	if (channel <= QC_MAX_CHANNEL)
	{
		qc_SelectChannel(channel);
		spi_WriteByte(commandByte);
		qc_DeselectChannel(channel);
	}
}
