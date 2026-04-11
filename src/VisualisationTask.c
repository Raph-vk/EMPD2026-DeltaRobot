/*
 * VisualisationTask.c
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// library & HAL includes

#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"
#include "I2CLib.h"  //voor scherm intergratie


///////////////////////////////////////////////////////////////////////////////
// application includes

#include "ApplicationTasks.h"
#include "ControlTask.h"
#include "VisualisationTask.h"
#include "bits.h"

///////////////////////////////////////////////////////////////////////////////
// void port_AllLampsOff(void)

void port_AllLampsOff(void)
{
	port_SetBit(LAMP_GREEN, false);
	port_SetBit(LAMP_ORANGE, false);
	port_SetBit(LAMP_RED, false);
}

///////////////////////////////////////////////////////////////////////////////
// void port_SetLamp(uint8_t lampNumber)
// Zet een lamp aan
//
void port_SetLamp(uint8_t lampNumber)
{
	
	if (lampNumber == LAMP_GREEN)
	{
		port_SetBit(LAMP_GREEN, true);
	}
	else if (lampNumber == LAMP_ORANGE)
	{
		port_SetBit(LAMP_ORANGE, true);
	}
	else if (lampNumber == LAMP_RED)
	{
		port_SetBit(LAMP_RED, true);
	}
	else
	{
		// ongeldige keuze -> alle lampen uit
	}
}

///////////////////////////////////////////////////////////////////////////////
// void VisualisationTask(void *pvParameters)
//
void VisualisationTask(void *pvParameters)
{
    SystemState_t status = STATE_INIT;
	SystemState_t previousStatus = (SystemState_t)-1;
	uint32_t blinkDelay = 200;			// knipperperiode in ms
	uint32_t i = 0;

	vPrintString("> starting VisualisationTask\n");
		
	// Laat control taak weten dat deze task gestart is.
	xEventGroupSetBits( handle_ThreadEventGroup, BIT_1 );


	/////////////////////////////////////////////////////////////////////////
	// Oneindige while-loop
	while (true)
	{
		// Huidige status ophalen
		xQueuePeek(handle_StateQueue, &status, 0);
		
		//Indien de status is veranderd, lampen uit en updaten.
		if (status != previousStatus)
		{
			port_AllLampsOff();
			previousStatus = status;
			i = 0;
		}
		
		// Uitvoer actie bij bepaalde state.
		switch (status)
		{
			case STATE_INIT:
			{			
				// Rood en oranje knipperen
				if (i >= 20) 
					port_AllLampsOff();
				else
				{
					port_SetLamp(LAMP_ORANGE);
					port_SetLamp(LAMP_RED);
				}
				
				//strcpy(line1, "System");
				//strcpy(line2, "Initialising...");
				//updateDisplay = true;
				
				break;
			}


			case STATE_WAIT:
			{
				//Rode lamp aan
				port_SetLamp(LAMP_RED);
				break;	
			}


			case STATE_HOMING:
			{
				//Orange lamp knipperen
				if (i >= 20)
					port_AllLampsOff();
				else
					port_SetLamp(LAMP_ORANGE);
				
				break;
			}


			case STATE_READY:
			{
				//Groene lamp aan
				port_SetLamp(LAMP_GREEN);
				break;
			}

			
			case STATE_RUNNING:
			{
				// Groene lamp knipperen
				if (i >= 20)
					port_AllLampsOff();
				else
					port_SetLamp(LAMP_GREEN);
				break;			
			}

			case STATE_FAULT:
			{
				//Rode lamp aan
				port_SetLamp(LAMP_RED);
				break;				
			}
		}
		taskSleep(10);
		
		//Loop teller voor blink, praktisch; 37, 38, 39, 0, 1, 2 ....
		i = (i + 1) % 40;;
		
		// TODO: NAAR OLED SCHERM SCHRIJVEN
		/* 
		Voorbeeld om naar OLED scherm te schrijven nog uitwerken
		
		displayTick++;
		if ((status != previousDisplayStatus) || (displayTick >= 50)) // Bij verandering of 10 ms per loop keer 50 = ongeveer 500 ms update.
		{
			OLED_Clear();
			OLED_WriteString(0, 0, line1);
			OLED_WriteString(0, 16, line2);
			OLED_Update();
			
			previousDisplayStatus = status;
			displayTick = 0;
		}
		*/
	}
	
	
	/* Should never get here */
}