/*
 * DAC4921Lib.h
 *
 * Created: 2-10-2022 15:41:15
 *  Author: Roel Smeets
 */ 


#ifndef DAC4921LIB_H_
#define DAC4921LIB_H_


///////////////////////////////////////////////////////////////////////////////
// defines for DAC MCP4921

#define DAC_SELECT_A     	 0x0000  // bit 15 == 0: set to DAC A
#define DAC_SELECT_B      	 0x8000  // bit 15 == 1: set to DAC B

#define DAC_VREF_UNBUFFERED  0x0000  // bit 14 == 0: unbuffered Vref input
#define DAC_VREF_BUFFERED    0x4000  // bit 14 == 1: buffered Vref input

#define DAC_GAINSELECT_1     0x2000  // bit 13 == 1: output gain = 1
#define DAC_GAINSELECT_2     0x0000  // bit 13 == 0: output gain = 2

#define DAC_POWER_DOWN       0x0000  // bit 12 == 0: disable output buffer, output is Hi-Z
#define DAC_POWER_ON         0x1000  // bit 12 == 1: enable output

#define DAC_MIN_VALUE		 0
#define DAC_MAX_VALUE		 4095

#define DAC_N_CHANNELS		4
#define DAC_MAX_CHANNEL		(DAC_N_CHANNELS - 1)

///////////////////////////////////////////////////////////////////////////////
// DAC voltage definitions on RTSW board

#define DAC_REFERENCE_VOLTAGE	( 2.5 )
#define DAC_MAX_CHIP_VOUT		( (4095.0/4096.0) * DAC_REFERENCE_VOLTAGE )
#define DAC_MAX_OUTPUTVOLTAGE	( -10.0 + (8.0*DAC_MAX_CHIP_VOUT) )	
#define DAC_MIN_OUTPUTVOLTAGE	( -10.0 )

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void dac_Init(void);
void dac_SelectChannel(uint8_t dacChannel);
void dac_DeselectChannel(uint8_t dacChannel);
uint32_t dac_Write(uint8_t dacChannel, uint16_t dacValue);

void dac_SelectAllChannels(void);
void dac_DeselectAllChannels(void);
void dac_WriteAll(uint16_t dacValue);

void dac_SetOutputVoltage(uint8_t dacChannel, float outputVoltage);

#endif /* DAC4921LIB_H_ */