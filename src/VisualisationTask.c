/*
 * VisualisationTask.c
 *
 * Created: 10-04-2026
 *  Author: Raph van Koeveringe
 */ 
///////////////////////////////////////////////////////////////////////////////
#include "VisualisationTask.h"

static uint8_t u8x8_byte_due_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static uint8_t u8x8_gpio_and_delay_due(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static Bool OLED_Recover(void);

///////////////////////////////////////////////////////////////////////////////
// file globals
static u8g2_t g_u8g2;
static I2C_HANDLE *g_oledI2c = NULL;
static Bool g_oledReady = false;
static Bool g_oledTransferError = false;
static status_code_t g_oledLastTransferStatus = STATUS_OK;
static uint32_t g_oledRecoverCount = 0;
static uint32_t g_oledTransferErrorCount = 0;

#define OLED_I2C_CHANNEL         I2C_CHANNEL_0	// SDA1/SCL1 on the Due
#define OLED_I2C_ADDRESS         (0x3C << 1)	// screen Adress
#define OLED_I2C_BLOCK_TIME_MS   20U

///////////////////////////////////////////////////////////////////////////////
// void port_AllLampsOff(void)

void port_AllLampsOff(void)
{
	//Alle lampen uit.
	port_SetBit(LAMP_GREEN, false);
	port_SetBit(LAMP_ORANGE, false);
	port_SetBit(LAMP_RED, false);
	led_DisplayValue(0x00); 
}

///////////////////////////////////////////////////////////////////////////////
// void port_SetLamp(uint8_t lampNumber)
// Zet een lamp aan
//
static void port_SetLamps(Bool green, Bool orange, Bool red)
{
	port_SetBit(LAMP_GREEN,  green);
	port_SetBit(LAMP_ORANGE, orange);
	port_SetBit(LAMP_RED,    red);

	uint8_t ledValue = 0x00;

	if (green)  ledValue |= 0x01;
	if (orange) ledValue |= 0x02;
	if (red)    ledValue |= 0x04;

	led_DisplayValue(ledValue);
}

///////////////////////////////////////////////////////////////////////////////
// OLED_Init(void)
//
Bool OLED_Init(void)
{
	u8g2_Setup_ssd1309_i2c_128x64_noname0_1(&g_u8g2, U8G2_R0, u8x8_byte_due_i2c, u8x8_gpio_and_delay_due);
	u8g2_SetI2CAddress(&g_u8g2, OLED_I2C_ADDRESS);

	g_oledTransferError = false;
	g_oledLastTransferStatus = STATUS_OK;
	g_oledRecoverCount = 0;
	g_oledTransferErrorCount = 0;

	g_oledReady = OLED_Recover();
	return g_oledReady;
}

///////////////////////////////////////////////////////////////////////////////
// Bool OLED_DrawStatus()
//
// strings naar de juiste positie op scherm schrijven.
//
Bool OLED_DrawStatus(const char *stateTxt, const char *opTxt1, const char *opTxt2, const char *stroomTxt, const char *potTxt)
{
	if (g_oledReady == false)
	{
		return false;
	}

	g_oledTransferError = false;
	g_oledLastTransferStatus = STATUS_OK;

	u8g2_FirstPage(&g_u8g2);
	do
	{
		// Bovenste STATUS regel
		u8g2_SetFont(&g_u8g2, u8g2_font_6x10_tf);
		u8g2_DrawStr(&g_u8g2, 0, 10, stateTxt);

		// operator compact
		u8g2_SetFont(&g_u8g2, u8g2_font_5x7_tr);
		u8g2_DrawStr(&g_u8g2, 0, 21, opTxt1);
		u8g2_DrawStr(&g_u8g2, 0, 31, opTxt2);

		// Onderste data regel
		u8g2_SetFont(&g_u8g2, u8g2_font_5x7_tr);
		u8g2_DrawStr(&g_u8g2, 0, 62,  stroomTxt);
		u8g2_DrawStr(&g_u8g2, 80, 62, potTxt);
	} while (u8g2_NextPage(&g_u8g2));

	if (g_oledTransferError)
	{
		g_oledReady = false;
		g_oledTransferErrorCount++;
		return false;
	}

	return true;
}


///////////////////////////////////////////////////////////////////////////////
// void VisualisationTask(void *pvParameters)
//
void VisualisationTask(void *pvParameters)
{
	//(void)pvParameters;
	vPrintString("> starting VisualisationTask\n");

    SystemState_t status = STATE_INIT;
	SystemState_t vorigeStatus = (SystemState_t)-1;
	uint32_t i = 0;

	char potLine[24];
	char stroomLine[24];
	char stateLine[24];
	const char *stateString = ""; //Moeten const en pointer zijn!
	const char *operatorLine1 = "";
	const char *operatorLine2 = "";
	Bool updateDisplay = false;
	
	//screen setup
	if (OLED_Init() == false)
	{
		vPrintString("> OLED init failed, display recovery will retry in task loop.\n");
	}

	// Laat control taak weten dat deze task gestart is.
	vPrintString("> running VisualisationTask\n");
	xEventGroupSetBits( handle_ThreadEventGroup , BIT_1 );

	/////////////////////////////////////////////////////////////////////////
	// Oneindige while-loop
	while (true)
	{
		// Huidige status ophalen
		xQueuePeek(handle_StateQueue, &status, 0);
		
		//Indien de status is veranderd, lampen uit en updaten.
		if (status != vorigeStatus)
		{
			port_AllLampsOff();
			vorigeStatus = status;
			updateDisplay = true;
			i = 0;
		}

		// Iedere loop setten
		stateString = "";
		operatorLine1 = "";
		operatorLine2 = "";
		Bool blink = (i < 5);
			
		// Uitvoer actie bij bepaalde state.
		switch (status)
		{
			case STATE_INIT:
			{			
				port_SetLamps(false, blink, false);
				
				stateString = "INITIALISEREN";
				operatorLine1 = "Wacht A.U.B.";
				break;
			}


			case STATE_WAIT:
			{
				//Rode lamp aan
				port_SetLamps(false, true, false);
				
				stateString = "WACHTEN";
				operatorLine1 = "Druk <START> om armen";
				operatorLine2 = "te refereren.";

				break;	
			}

			case STATE_HOMING:
			{
				//Orange lamp knipperen
				port_SetLamps(false, blink, false);

				stateString = "REFEREREN";
				operatorLine1 = "Wacht A.U.B.";

				break;
			}

			case STATE_READY:
			{			
				port_SetLamps(true, false, false);
				
				stateString = "Gereed";
				operatorLine1 = "Druk <START> om te";
				operatorLine2 = "beginnen.";
				break;
			}

			case STATE_PAUSE:
			{
				port_SetLamps(true, true, false);
				
				stateString = "PAUZE";
				operatorLine1 = "<START> om te hervatten,";
				operatorLine2 = "<RESET> om aftebreken.";
				break;
			}

			
			case STATE_RUNNING:
			{
				// Groene lamp knipperen
				port_SetLamps(blink, false, false);
					
				stateString = "ACTIEF";
				operatorLine1 = "Druk <STOP> om cyclus";
				operatorLine2 = "te pauzeren.";
				break;			
			}

			case STATE_FAULT:
			{
				stateString = "FOUT";
				
				//Rode lamp aan
				port_SetLamps(false, false, true);
				operatorLine1 = "Herstel noodsignaal en";
				operatorLine2 = "druk <RESET> knop in.";
				break;				
			}
		}
		
		// NAAR OLED SCHERM SCHRIJVEN
		if (!updateDisplay || (i < 10))
		{
			//Loop teller voor blink, praktisch; 7, 8, 9, 0, 1, 2 ....
			i++;
			taskSleep(100);
		}
		else
		{
			uint32_t procent;
			float stroom;
			
			// Potmeterwaarde niet-destructief uit de queue lezen
			if (xQueuePeek(handle_potQueue, &procent, 0) == pdTRUE)
			{
				snprintf(potLine, sizeof(potLine), "vMax:%0u%%", (unsigned int)procent);
			}
			else
			{
				snprintf(potLine, sizeof(potLine), "vMax:---");
			}
			
			// Stroomwaarde niet-destructief uit de queue lezen
			if (xQueuePeek(handle_stroomQueue, &stroom, 0) == pdTRUE)
			{
				snprintf(stroomLine, sizeof(stroomLine), "Stroom:%.1fA", stroom); //1decimaal
				vPrintString("Gemeten stroom: %.1fA\n", stroom);
			}
			else
			{
				snprintf(stroomLine, sizeof(stroomLine), "Stroom:---A");
			}
			
			//Infolijn TOEVOEGEN.
			snprintf(stateLine, sizeof(stateLine), "Status: %s", stateString);
			if (OLED_DrawStatus(stateLine, operatorLine1, operatorLine2, stroomLine, potLine))
			{
				updateDisplay = false;
			}
			else
			{
				vPrintString("> OLED transfer failed (%ld), trying recovery.\n", (long)g_oledLastTransferStatus);
				(void)OLED_Recover();
				updateDisplay = true;
			}
		}
	}
	/* Should never get here */
}






///////////////////////////////////////////////////////////////////////////////
// SCREEN FUNCTIONS
//
static Bool OLED_Recover(void)
{
	uint8_t clearResult;

	g_oledI2c = i2c_GetChannelHandle(OLED_I2C_CHANNEL);
	if (g_oledI2c == NULL || g_oledI2c->twi == NULL)
	{
		vPrintString("> OLED recover failed: invalid I2C handle.\n");
		return false;
	}

	clearResult = i2c_ClearBus(OLED_I2C_CHANNEL);
	if ((clearResult != I2C_BUS_OK) && (clearResult != I2C_BUS_REPAIRED))
	{
		vPrintString("> OLED recover failed: %s\n", i2c_GetErrorMessage(clearResult));
		return false;
	}

	u8g2_InitDisplay(&g_u8g2);
	u8g2_SetPowerSave(&g_u8g2, 0);
	u8g2_SetFont(&g_u8g2, u8g2_font_5x7_tr);

	g_oledTransferError = false;
	g_oledLastTransferStatus = STATUS_OK;
	g_oledReady = true;
	g_oledRecoverCount++;

	vPrintString("> OLED recovered (%lu), bus=%s\n",
	(unsigned long)g_oledRecoverCount,
	i2c_GetErrorMessage(clearResult));

	return true;
}


static uint8_t u8x8_byte_due_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	static uint8_t txBuffer[132];
	static uint8_t txIndex = 0;
	portTickType maxBlockTimeTicks = (OLED_I2C_BLOCK_TIME_MS / portTICK_RATE_MS);
	twi_packet_t packet;
	status_code_t writeStatus;
	uint8_t *data;

	switch (msg)
	{
		case U8X8_MSG_BYTE_INIT:
		g_oledI2c = i2c_GetChannelHandle(OLED_I2C_CHANNEL);
		return ((g_oledI2c != NULL) && (g_oledI2c->twi != NULL));

		case U8X8_MSG_BYTE_START_TRANSFER:
		txIndex = 0;
		return 1;

		case U8X8_MSG_BYTE_SEND:
		data = (uint8_t *)arg_ptr;
		while (arg_int > 0)
		{
			if (txIndex < sizeof(txBuffer))
			{
				txBuffer[txIndex++] = *data;
			}
			data++;
			arg_int--;
		}
		return 1;

		case U8X8_MSG_BYTE_END_TRANSFER:
		if (g_oledI2c == NULL || g_oledI2c->twi == NULL || txIndex == 0)
		{
			return 0;
		}
		memset(&packet, 0, sizeof(packet));
		packet.chip = u8x8_GetI2CAddress(u8x8) >> 1;
		packet.addr_length = 0;
		packet.buffer = txBuffer;
		packet.length = txIndex;
		writeStatus = freertos_twi_write_packet(g_oledI2c->twi, &packet, maxBlockTimeTicks);
		if (writeStatus != STATUS_OK)
		{
			g_oledLastTransferStatus = writeStatus;
			g_oledTransferError = true;
			return 0;
		}
		return 1;

		default:
		return 1;
	}
}

static uint8_t u8x8_gpio_and_delay_due(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch (msg)
	{
		case U8X8_MSG_GPIO_AND_DELAY_INIT:
		break;

		case U8X8_MSG_DELAY_MILLI:
		delay_ms(arg_int);
		break;

		case U8X8_MSG_DELAY_10MICRO:
		delay_us(10);
		break;

		case U8X8_MSG_DELAY_100NANO:
		break;

		case U8X8_MSG_GPIO_RESET:
		break;
	}

	return 1;
}