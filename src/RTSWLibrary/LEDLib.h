/*
 * LEDLib.h
 *
 * Created: 13-10-2022 14:38:24
 *  Author: rasmsmee
 */ 


#ifndef LEDLIB_H_
#define LEDLIB_H_

///////////////////////////////////////////////////////////////////////////////
// defines

#define N_LEDS	4

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void led_Init(void);
void led_DisplayValue(uint8_t value);
bool led_IsValidNumber(uint8_t ledNumber);
void led_SetState(uint8_t ledNumber, bool ledOn);

#endif /* LEDLIB_H_ */