/*
 * VisualisationTask.c
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */
#include "VisualisationTask.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// library & HAL includes
#include "vPrintString.h"
#include "TaskSleep.h"

#include "bits.h" // voor lampen
#include "LEDLib.h"
#include "PortIOLib.h"

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "ApplicationTasks.h"
#include "ControlTask.h"
#include "MachinePins.h"
#include "Screen.h"

///////////////////////////////////////////////////////////////////////////////
// void port_AllLampsOff(void)
/*
 * Schakelt alle statuslampen en board-LEDs uit.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; alle lampuitgangen worden laag gezet.
 */
static void port_AllLampsOff(void)
{
	// Alle lampen uit.
	port_SetBit(BIT_LAMP_GREEN, false);
	port_SetBit(BIT_LAMP_ORANGE, false);
	port_SetBit(BIT_LAMP_RED, false);
	led_DisplayValue(0x00);
}

///////////////////////////////////////////////////////////////////////////////
// static void port_SetLamps(bool green, bool orange, bool red)
/*
 * Zet de groene, oranje en rode statuslamp op de gevraagde toestand.
 * Invoer: green, orange en red bepalen per lamp of deze aan moet.
 * Uitvoer: geen returnwaarde; schrijft naar de lampuitgangen en LED-indicatie.
 */
static void port_SetLamps(bool green, bool orange, bool red)
{
	port_SetBit(BIT_LAMP_GREEN,  green);
	port_SetBit(BIT_LAMP_ORANGE, orange);
	port_SetBit(BIT_LAMP_RED,    red);

	uint8_t ledValue = 0x00;

	if (green)  ledValue |= 0x01;
	if (orange) ledValue |= 0x02;
	if (red)    ledValue |= 0x04;

	led_DisplayValue(ledValue);
}

///////////////////////////////////////////////////////////////////////////////
// void VisualisationTask(void *pvParameters)
/*
 * Toont de actuele systeemstatus via lampen, LEDs en het OLED-scherm.
 * Invoer: pvParameters wordt niet gebruikt.
 * Uitvoer: geen returnwaarde; de taak blijft periodiek visualisaties bijwerken.
 */
void VisualisationTask(void *pvParameters)
{
	//(void)pvParameters;
	vPrintString("> starting VisualisationTask\n");

	SystemState_t status = STATE_INIT;
	SystemState_t vorigeStatus = (SystemState_t)-1;
	uint32_t i = 0;

	char stateLine[24];
	const char *stateString = ""; // Moeten const en pointer zijn!
	
	
	char tcpLine[10];
	bool TCPcomp = false;
	bool lastTCPcomp = true;
	
	static char overigeInfo1[DISPLAY_INFO_LINE_LENGTH] = "";
	static char overigeInfo2[DISPLAY_INFO_LINE_LENGTH] = "";
	DisplayInfo_t displayInfo;
	const char *operatorLine1 = "";
	const char *operatorLine2 = "";
	bool updateDisplay = true;   // eerste schermwrite expliciet aanvragen
	bool blink = false;

	// Screen setup
	if (Screen_Init() == false)
	{
		vPrintString("> OLED init failed, display recovery will retry in task loop.\n");
	}

	// Laat control taak weten dat deze task gestart is.
	vPrintString("> running VisualisationTask\n");
	xEventGroupSetBits(handle_ThreadEventGroup, BIT_1);

	/////////////////////////////////////////////////////////////////////////
	// Oneindige while-loop
	while (true)
	{
		// Huidige status ophalen
		xQueuePeek(handle_StateQueue, &status, 0);

		// Indien de status is veranderd, lampen uit en updaten.
		if (status != vorigeStatus)
		{
			port_AllLampsOff();
			vorigeStatus = status;
			updateDisplay = true;
		}
		
		// Iedere loop setten
		stateString = "";
		operatorLine1 = "";
		operatorLine2 = "";
		blink = ((i % 10) < 5);

		// Uitvoer actie bij bepaalde state.
		switch (status)
		{
			case STATE_INIT:
			{
				port_SetLamps(false, true, true);

				stateString = "OPSTARTEN";
				operatorLine1 = "Wacht A.U.B.";
				break;
			}

			case STATE_WAIT:
			{
				port_SetLamps(false, blink, false);

				stateString = "WACHTEN";
				operatorLine1 = "Druk <START> om armen";
				operatorLine2 = "te refereren.";
				break;
			}

			case STATE_HOMING:
			{
				port_SetLamps(false, true, false);

				stateString = "HOMING";
				operatorLine1 = "Wacht A.U.B.";
				break;
			}

			case STATE_READY:
			{
				port_SetLamps(blink, false, false);

				stateString = "GEREED";
				operatorLine1 = "Druk <START> om te";
				operatorLine2 = "beginnen.";
				break;
			}

			case STATE_TEACH:
			{
				port_SetLamps(blink, blink, false);

				stateString = "TEACH";
				operatorLine1 = "UserFrame inmeten";
				operatorLine2 = " ";
				break;
			}


			case STATE_RUNNING:
			{
				// Groene lamp aan
				port_SetLamps(true, false, false);

				stateString = "ACTIEF";
				operatorLine1 = "Druk <STOP> om cyclus";
				operatorLine2 = "te pauzeren.";
				break;
			}

			case STATE_PAUSE:
			{
				port_SetLamps(blink, blink, false);

				stateString = "PAUZE";
				operatorLine1 = "<START> om te hervatten,";
				operatorLine2 = "<RESET> om aftebreken.";
				break;
			}

			case STATE_FAULT:
			{
				stateString = "FOUT";

				// Rode lamp aan
				port_SetLamps(false, false, true);
				operatorLine1 = "Herstel noodsignaal en";
				operatorLine2 = "druk <RESET> knop in.";
				break;
			}
		}
		
		
		// Laatste algemene scherminformatie ophalen voor de onderste schermregels.
		if ((handle_DisplayInfoQueue != NULL) && (xQueueReceive(handle_DisplayInfoQueue, &displayInfo, 0) == pdTRUE))
		{
			snprintf(overigeInfo1, sizeof(overigeInfo1), "%s", displayInfo.regel1);
			snprintf(overigeInfo2, sizeof(overigeInfo2), "%s", displayInfo.regel2);

			updateDisplay = true;
		}
		

		//controleren of TCPcompensation is veranderd
		TCPcomp = TcpCompensation_IsEnabled();
		if (TCPcomp != lastTCPcomp)
		{
			snprintf(tcpLine, sizeof(tcpLine), "TCP:%s", TCPcomp ? "AAN" : "UIT");
			lastTCPcomp = TCPcomp;
			updateDisplay = true;
		}

		// NAAR OLED SCHERM SCHRIJVEN
		if (updateDisplay)
		{
			// Infolijn toevoegen.
			snprintf(stateLine, sizeof(stateLine), "Status: %s", stateString);
			//snprintf(tcpLine, sizeof(tcpLine), "TCP:%s", TcpCompensation_IsEnabled() ? "AAN" : "UIT");
			
			if (Screen_DrawStatus(stateLine, tcpLine, operatorLine1, operatorLine2, overigeInfo1, overigeInfo2))
			{
				updateDisplay = false;
			}
			else
			{
				//vPrintString("> OLED transfer failed (%ld), trying recovery.\n", (long)Screen_GetLastTransferStatus());
				(void)Screen_Recover();
				updateDisplay = true;
			}
		}//end-screen if-statement
		else
		{
			// Loop teller voor blink, praktisch; 7, 8, 9, 0, 1, 2 ....
			i++;
		}
		taskSleep(100);
	}
	/* Should never get here */
}
