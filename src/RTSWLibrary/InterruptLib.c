/*
 * InterruptLib.c
 *
 * Created: 16-10-2022 08:39:14
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "InterruptLib.h"


///////////////////////////////////////////////////////////////////////////////
// void interrupt_Disable(due_pin_number_t duePinID)

void interrupt_Disable(due_pin_number_t duePinID)
{
	uint32_t ioPortPinNumber = 0;
	
	ioPortPinNumber	= dio_GetIOPortPinNumber(duePinID);
	pio_disable_interrupt(arch_ioport_pin_to_base(ioPortPinNumber), arch_ioport_pin_to_mask(ioPortPinNumber));
}

void interrupt_Enable(due_pin_number_t duePinID)
{
	uint32_t ioPortPinNumber = 0;
	
	ioPortPinNumber	= dio_GetIOPortPinNumber(duePinID);
	pio_enable_interrupt(arch_ioport_pin_to_base(ioPortPinNumber), arch_ioport_pin_to_mask(ioPortPinNumber));
}


///////////////////////////////////////////////////////////////////////////////
// void interrupt_AttachHandler(INTERRUPTHANDLER handler, 
//								due_pin_number_t duePinID, uint32_t flags)

void interrupt_AttachHandler(INTERRUPTHANDLER handler, due_pin_number_t duePinID, uint32_t flags)
{
	PIO_INT_HANDLE *pioInterruptHandle = NULL;
	
	uint32_t ioPortPinNumber = 0;
	
	ioPortPinNumber	   = dio_GetIOPortPinNumber(duePinID);
	pioInterruptHandle = pioSetInterruptHandlerOnPin(handler, ioPortPinNumber, flags);

	pioEnableInterrupt(pioInterruptHandle);
}


/////////////////////////////////////////////////////////////////////////////// 
// PIO_INT_HANDLE *pioSetInterruptHandlerOnPin(
//		INTERRUPTHANDLER handler, ioport_pin_t pin, uint32_t interruptPinFlags)
//
// interruptPinFlags:
//		PIO_IT_FALL_EDGE, PIO_IT_RISE_EDGE, PIO_IT_LOW_LEVEL of PIO_IT_HIGH_LEVEL

PIO_INT_HANDLE *pioSetInterruptHandlerOnPin(INTERRUPTHANDLER handler, ioport_pin_t pin, uint32_t interruptPinFlags)
{

	uint32_t deviceId  = 0xffffffff;
	IRQn_Type irq_type = -100;
	PIO_INT_HANDLE *handle = NULL;

	ioport_enable_pin(pin);
	ioport_set_pin_dir(pin,  IOPORT_DIR_INPUT);
	ioport_set_pin_mode(pin, IOPORT_MODE_PULLUP);	// with pull-up

	uint32_t ioPortId = (uint32_t) ((uint32_t *)arch_ioport_pin_to_base(pin));
	
	switch(ioPortId)
	{
		case (int)PIOA:
			deviceId = ID_PIOA;
			irq_type = PIOA_IRQn;
			break;
		
		case (int)PIOB:
			deviceId = ID_PIOB;
			irq_type = PIOB_IRQn;
			break;
		
		case (int)PIOC:
			deviceId = ID_PIOC;
			irq_type = PIOC_IRQn;
			break;
		
		case (int)PIOD:
			deviceId = ID_PIOD;
			irq_type = PIOD_IRQn;
			break;
		
		default:
			return NULL;
			break;
	}

	pmc_enable_periph_clk(deviceId);
	
	handle = malloc(sizeof(PIO_INT_HANDLE));
	handle->pin = pin;

	pioDisableInterrupt(handle);
	
	pio_handler_set(arch_ioport_pin_to_base(pin), deviceId,
		arch_ioport_pin_to_mask(pin), interruptPinFlags, handler);
	
	// set NVIC_SetPriority to 1 LOWER then configMAX_SYSCALL_INTERRUPT_PRIORITY !!!!!
	// RASM, 22-12-01
	NVIC_SetPriority(irq_type, configMAX_SYSCALL_INTERRUPT_PRIORITY - 1);
	
	//enable the PIO interrupt in the NVIC:
	NVIC_EnableIRQ(irq_type); 
	
	return handle;
}


void pioEnableInterrupt(PIO_INT_HANDLE *handle)
{
	if (handle != NULL)
	{
		pio_enable_interrupt(arch_ioport_pin_to_base(handle->pin), arch_ioport_pin_to_mask(handle->pin));
	}
}


void pioDisableInterrupt(PIO_INT_HANDLE *handle)
{
	if (handle != NULL)
	{
		pio_disable_interrupt(arch_ioport_pin_to_base(handle->pin), arch_ioport_pin_to_mask(handle->pin));
	}
}
