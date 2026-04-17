/*
 * MachinePins.h
 *
 * Created: 12/04/2026 11:35:37
 *  Author: raphv
 */ 

#ifndef MACHINEPINS_H_
#define MACHINEPINS_H_

#include "DeviceIOLib.h"
#include "PortIOLib.h"
#include "SwitchLib.h"

///////////////////////////////////////////////////////////////////////////////
// Hoeveelheid motoren
#define N_MOTORS		3
#define i_twk			(47.0f) // 47:1 reductie tandwielkast

///////////////////////////////////////////////////////////////////////////////
// 8-bit input port layout on the RTSW board (handeld by PortIOLib.h)

#define BIT_CLOCK			0 // PIN_30
#define BIT_M1_HOME			1 // PIN_31
#define BIT_M2_HOME			2 // PIN_32
#define BIT_M3_HOME			3 // PIN_33
#define BIT_NOOD			4 // PIN_34
#define BIT_START			5 // PIN_35
//#define BIT_STOP  6 // PIN_36
//#define BIT_RESET  7

//PINS for Attach_InterruptHandler() function
#define PIN_CLOCK			PIN_30
#define PIN_NOOD			PIN_34
//#define PIN_NOODSTOP		PIN_35
//#define PIN_NOODSCHAKELAAR  PIN_36

///////////////////////////////////////////////////////////////////////////////
// 8-bit output port layout on the RTSW board (handeld by PortIOLib.h)

//#define BIT_ESCON_ENABLE    0 // PIN_38 (alle escon's aan)
#define BIT_LAMP_GREEN		1 // PIN_39
#define BIT_LAMP_ORANGE		2 // PIN_40
#define	BIT_LAMP_RED		3 // PIN_41
//#define	4 // PIN_42
//#define   5 // PIN_43
//#define   6 // PIN_44
//#define   7 // PIN_45

///////////////////////////////////////////////////////////////////////////////
// I2C ports
// Scherm1
// Scherm2

///////////////////////////////////////////////////////////////////////////////
// DAC -> Escon (ook via SPI)
// 3x DAC escon

///////////////////////////////////////////////////////////////////////////////
// PCB push buttons (handled by SwitchLib)

// Hierop ook de echte drukknoppen op aansluiten?
#define PCB_SWITCH_START        0 // PIN_A9 
#define PCB_SWITCH_STOP         1 // PIN_A8
#define PCB_SWITCH_RESET        2 // PIN_A7
//#define PCB_SWITCH_			3 // PIN_A6 // Wellicht potmeter toevoegen? of op andere Apin


#endif /* MACHINEPINS_H_ */