/*
 * SPILib.c
 *
 * Created: 4-10-2022 22:39:12
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "SPILib.h"


///////////////////////////////////////////////////////////////////////////////
// file global objects

static struct spi_device spiDevice;

static bool G_IsSPIInitialised = false;

///////////////////////////////////////////////////////////////////////////////
// void spi_Init(void)

void spi_Init(void)
{
	if (G_IsSPIInitialised == false)	// prevents multiple inits
	{
		spi_master_init(SPI0);
		spi_master_setup_device(SPI0, &spiDevice, SPI_MODE_0, SPI_BAUDRATE, 0);
		spi_enable(SPI0);
		G_IsSPIInitialised = true;
	}
}

///////////////////////////////////////////////////////////////////////////////
// bool spi_OperationCompleted(void)

bool spi_OperationCompleted(void)
{
	bool completed = false;
	
	if (spi_is_tx_empty(SPI0))
	{
		completed = true;
	}
	else
	{
		completed = false;
	}
	
	return completed;
}

///////////////////////////////////////////////////////////////////////////////
// status_code_t spi_WritePacket(const uint8_t *data, size_t numBytes)

status_code_t spi_WritePacket(const uint8_t *data, size_t numBytes)
{
	status_code_t status = OPERATION_IN_PROGRESS;
	
	status = spi_write_packet(SPI0, data, numBytes);
	
	while (spi_OperationCompleted() == false)
	{
	}	
	
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// status_code_t spi_WriteByte(const uint8_t data)

status_code_t spi_WriteByte(const uint8_t data)
{
	uint8_t spidata = data;
	status_code_t status = OPERATION_IN_PROGRESS;
	
	status = spi_WritePacket(&spidata, 1);
	
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// status_code_t spi_WriteWord(const uint16_t data)

status_code_t spi_WriteWord(const uint16_t data)
{
	uint8_t spidata[2];
	status_code_t status = OPERATION_IN_PROGRESS;
	
	spidata[0] = (data & 0xff00) >> 8;	// high byte
	spidata[1] = (data & 0x00ff);		// low byte
	
	status = spi_WritePacket(spidata, 2);
	
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// status_code_t spi_ReadPacket(uint8_t *data, size_t numBytes)

status_code_t spi_ReadPacket(uint8_t *data, size_t numBytes)
{
	status_code_t status = OPERATION_IN_PROGRESS;
	
	status = spi_read_packet(SPI0, data, numBytes);
	
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// status_code_t spi_WriteByte(const uint8_t data)

status_code_t spi_ReadByte(uint8_t *data)
{
	uint8_t spidata = 0xff;
	status_code_t status = OPERATION_IN_PROGRESS;
	
	status = spi_ReadPacket(&spidata, 1);
	
	*data = spidata;
	
	return status;
}