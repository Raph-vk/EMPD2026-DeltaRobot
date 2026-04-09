/*
 * SPILib.h
 *
 * Created: 4-10-2022 22:41:52
 *  Author: Roel Smeets
 */ 


#ifndef SPILIB_H_
#define SPILIB_H_

///////////////////////////////////////////////////////////////////////////////
// defines

#define SPI_BAUDRATE	4000000UL

///////////////////////////////////////////////////////////////////////////////
//	function prototypes

void spi_Init(void);
bool spi_OperationCompleted(void);

status_code_t spi_WritePacket(const uint8_t *data, size_t numBytes);
status_code_t spi_WriteByte(const uint8_t data);
status_code_t spi_WriteWord(const uint16_t data);

status_code_t spi_ReadPacket(uint8_t *data, size_t numBytes);
status_code_t spi_ReadByte(uint8_t *data);

#endif /* SPILIB_H_ */