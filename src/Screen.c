
/*
 * Screen.c
 *
 * Created: 24/05/2026 16:50:14
 *  Author: raphv
 */ 
#include "Screen.h"

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <stdint.h>
#include <string.h> // nodig voor memset-fucntie

///////////////////////////////////////////////////////////////////////////////
// library & HAL includes
#include "vPrintString.h"
#include "I2CLib.h"
#include "u8g2.h"

///////////////////////////////////////////////////////////////////////////////
// function prototypes
static uint8_t u8x8_byte_due_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static uint8_t u8x8_gpio_and_delay_due(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

///////////////////////////////////////////////////////////////////////////////
// file globals
static u8g2_t g_u8g2;
static I2C_HANDLE *g_oledI2c = NULL;
static bool g_oledReady = false;
static bool g_oledTransferError = false;
static status_code_t g_oledLastTransferStatus = STATUS_OK;
static uint32_t g_oledRecoverCount = 0;
static uint32_t g_oledTransferErrorCount = 0;

#define OLED_I2C_CHANNEL         I2C_CHANNEL_0     // SDA1/SCL1 on the Due
#define OLED_I2C_ADDRESS         (0x3C << 1)       // screen address
#define OLED_I2C_BLOCK_TIME_MS   20U

///////////////////////////////////////////////////////////////////////////////
// bool Screen_Init(void)
/*
 * Zet het OLED-scherm en de u8g2 I2C-callbacks klaar.
 * Invoer: geen.
 * Uitvoer: true wanneer het scherm klaar is, anders false.
 */
bool Screen_Init(void)
{
	u8g2_Setup_ssd1309_i2c_128x64_noname0_1(&g_u8g2, U8G2_R0, u8x8_byte_due_i2c, u8x8_gpio_and_delay_due);
	u8g2_SetI2CAddress(&g_u8g2, OLED_I2C_ADDRESS);

	g_oledTransferError = false;
	g_oledLastTransferStatus = STATUS_OK;
	g_oledRecoverCount = 0;
	g_oledTransferErrorCount = 0;

	g_oledReady = Screen_Recover();
	return g_oledReady;
}

///////////////////////////////////////////////////////////////////////////////
// bool Screen_DrawStatus(const char *stateTxt, const char *opTxt1, const char *opTxt2, const char *overigeInfo1, const char *overigeInfo2)
/*
 * Schrijft de actuele statusregels naar vaste posities op het OLED-scherm.
 * Invoer: tekstregels voor status, operatorinformatie en overige informatie.
 * Uitvoer: true wanneer de transfer gelukt is, anders false.
 */
bool Screen_DrawStatus(const char *stateTxt, const char *tcpComp, const char *opTxt1, const char *opTxt2, const char *overigeInfo1, const char *overigeInfo2 )
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
		
		u8g2_SetFont(&g_u8g2, u8g2_font_5x7_tr);
		u8g2_DrawStr(&g_u8g2, 90, 10, tcpComp);
		
		// Operator compact
		u8g2_DrawStr(&g_u8g2, 0, 21, opTxt1);
		u8g2_DrawStr(&g_u8g2, 0, 31, opTxt2);

		// Onderste overige info
		u8g2_DrawStr(&g_u8g2, 0, 50, overigeInfo1);
		u8g2_DrawStr(&g_u8g2, 0, 60, overigeInfo2);

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
// bool Screen_Recover(void)
/*
 * Probeert de I2C-bus te herstellen en initialiseert het OLED-scherm opnieuw.
 * Invoer: geen.
 * Uitvoer: true wanneer herstel en initialisatie gelukt zijn, anders false.
 */
bool Screen_Recover(void)
{
	uint8_t clearResult;

	g_oledI2c = i2c_GetChannelHandle(OLED_I2C_CHANNEL);
	if (g_oledI2c == NULL || g_oledI2c->twi == NULL)
	{
		//vPrintString("> OLED recover failed: invalid I2C handle.\n");
		return false;
	}

	clearResult = i2c_ClearBus(OLED_I2C_CHANNEL);
	if ((clearResult != I2C_BUS_OK) && (clearResult != I2C_BUS_REPAIRED))
	{
		//vPrintString("> OLED recover failed: %s\n", i2c_GetErrorMessage(clearResult));
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

///////////////////////////////////////////////////////////////////////////////
// status_code_t Screen_GetLastTransferStatus(void)
/*
 * Geeft de laatste I2C/u8g2 transferstatus terug.
 * Invoer: geen.
 * Uitvoer: status_code_t van de laatst mislukte of uitgevoerde transfer.
 */
status_code_t Screen_GetLastTransferStatus(void)
{
	return g_oledLastTransferStatus;
}

///////////////////////////////////////////////////////////////////////////////
// static uint8_t u8x8_byte_due_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
/*
 * Byte-callback voor u8g2/u8x8 die displaydata via de FreeRTOS I2C-driver stuurt.
 * Invoer: u8x8 context, berichttype, argumentwaarde en databuffer.
 * Uitvoer: 1 bij geslaagde verwerking, 0 bij transferfout.
 */
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

///////////////////////////////////////////////////////////////////////////////
// static uint8_t u8x8_gpio_and_delay_due(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
/*
 * GPIO- en delay-callback voor u8g2/u8x8 op de Arduino Due.
 * Invoer: u8x8 context, berichttype, argumentwaarde en optionele pointer.
 * Uitvoer: 1 wanneer het bericht verwerkt is.
 */
static uint8_t u8x8_gpio_and_delay_due(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	(void)u8x8;
	(void)arg_ptr;

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
