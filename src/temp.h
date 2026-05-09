/*
 * temp.h
 *
 * Created: 08/05/2026 
 *  Author: raphv
 */ 

#ifndef TEMP_H_
#define TEMP_H_

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>

///////////////////////////////////////////////////////////////////////////////
// FreeRTOS includes

#include "CommandConsole.h"
#include "vPrintString.h"


///////////////////////////////////////////////////////////////////////////////
// functions declerations
void RunningLoopTimer_Init(void);
void RunningLoopTimer_ResetWindow(void);
void RunningLoopTimer_Begin(void);
void RunningLoopTimer_End(void);

#endif /* TEMP_H_ */