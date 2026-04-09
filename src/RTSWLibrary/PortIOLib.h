/*
 * PortIOLib.h
 *
 * Created: 13-10-2022 21:56:33
 *  Author: Roel Smeets
 */ 


#ifndef PORTIOLIB_H_
#define PORTIOLIB_H_


#define N_INPUT_PINS	8
#define N_OUTPUT_PINS	8

void port_Init(void);
uint8_t port_GetInput(void);
void port_WriteOutput(uint8_t value);
bool port_IsValidBitNumber(uint8_t bitNumber);
bool port_IsBitSet(uint8_t bitNumber);
void port_SetBit(uint8_t bitNumber, bool bitOn);

#endif /* PORTIOLIB_H_ */