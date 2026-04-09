/*
 * ADCLib.h
 *
 * Created: 2-10-2022 12:05:06
 *  Author: Roel Smeets
 */ 


#ifndef ADCLIB_H_
#define ADCLIB_H_

//////////////////////////////////////////////////////////////////////////////
// #define's

#define ADC_CLOCK	ADC_FREQ_MIN		// ADC Clock frequency

#define ADC_CHANNEL_0_EOC	ADC_ISR_EOC0	// End of conversion Channel 0
#define ADC_CHANNEL_1_EOC	ADC_ISR_EOC1	// End of conversion Channel 1
#define ADC_CHANNEL_2_EOC	ADC_ISR_EOC2	// End of conversion Channel 2
#define ADC_CHANNEL_3_EOC	ADC_ISR_EOC3	// End of conversion Channel 3
#define ADC_CHANNEL_4_EOC	ADC_ISR_EOC4	// End of conversion Channel 4
#define ADC_CHANNEL_5_EOC	ADC_ISR_EOC5	// End of conversion Channel 5
#define ADC_CHANNEL_6_EOC	ADC_ISR_EOC6	// End of conversion Channel 6
#define ADC_CHANNEL_7_EOC	ADC_ISR_EOC7	// End of conversion Channel 7


#define ADC_CHANNEL_8_EOC	ADC_ISR_EOC8	// End of conversion Channel 8
#define ADC_CHANNEL_9_EOC	ADC_ISR_EOC9	// End of conversion Channel 9
#define ADC_CHANNEL_10_EOC	ADC_ISR_EOC10	// End of conversion Channel 10
#define ADC_CHANNEL_11_EOC	ADC_ISR_EOC11	// End of conversion Channel 11
#define ADC_CHANNEL_12_EOC	ADC_ISR_EOC12	// End of conversion Channel 12
#define ADC_CHANNEL_13_EOC	ADC_ISR_EOC13	// End of conversion Channel 13

#define ADC_DATA_RDY		ADC_ISR_DRDY	// ADC Data Ready

#define ADC_MAX_CHANNEL		7

#define ADC_MIN_VALUE		0
#define ADC_MAX_VALUE		4095

//////////////////////////////////////////////////////////////////////////////
// function prototypes

uint8_t adc_MapChanneltoSAM(uint8_t dueChannelNumber);

void adc_Setup(void);

void adc_Init(void);
void adc_EnableChannel(uint8_t channel);
void adc_DisableChannel(uint8_t channel);
void adc_StartConversion(void);
bool adc_IsConversionReady(uint8_t channel);
bool adc_IsValidChannel(uint8_t channel);
uint32_t adc_ReadData(uint8_t channel);

float adc_Fmap(float x, float in_min, float in_max, float out_min, float out_max);
float adc_ValueToVoltage(uint16_t adcValue);

#endif /* ADCLIB_H_ */