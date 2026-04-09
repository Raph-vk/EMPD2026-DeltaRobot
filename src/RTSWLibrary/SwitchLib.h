/*
 * SwitchLib.h
 *
 * Created: 13-10-2022 19:12:38
 *  Author: Roel Smeets
 */ 


#ifndef SWITCHLIB_H_
#define SWITCHLIB_H_


#define N_SWITCHES	4

void switch_Init(void);
bool switch_IsValidNumber(uint8_t switchNumber);
bool switch_IsPressed(uint8_t switchNumber);
uint8_t switch_GetValue(void);


#endif /* SWITCHLIB_H_ */