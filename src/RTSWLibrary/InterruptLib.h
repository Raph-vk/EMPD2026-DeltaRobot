/*
 * InterruptLib.h
 *
 * Created: 16-10-2022 08:39:49
 *  Author: Roel Smeets
 */ 


#ifndef INTERRUPTLIB_H_
#define INTERRUPTLIB_H_

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "DeviceIOLib.h"


///////////////////////////////////////////////////////////////////////////////
// function typedefs

typedef void (*INTERRUPTHANDLER)(uint32_t deviceIdentifier, uint32_t pinIdentifier);


///////////////////////////////////////////////////////////////////////////////
// structure definitions

typedef struct
{
	ioport_pin_t pin;
} PIO_INT_HANDLE;


///////////////////////////////////////////////////////////////////////////////
// function prototypes

void interrupt_AttachHandler(INTERRUPTHANDLER handler, due_pin_number_t duePinID, uint32_t flags);
void interrupt_Enable(due_pin_number_t duePinID);
void interrupt_Disable(due_pin_number_t duePinID);

// internal helper functions

PIO_INT_HANDLE *pioSetInterruptHandlerOnPin(INTERRUPTHANDLER intHandler, ioport_pin_t ioPortPinId, uint32_t interruptPinFlags);
void pioEnableInterrupt( PIO_INT_HANDLE *handle);
void pioDisableInterrupt(PIO_INT_HANDLE *handle);


#endif /* INTERRUPTLIB_H_ */