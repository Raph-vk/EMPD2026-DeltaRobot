/*
 * main.c
 *
 * Created: 13-11-2023 19:36:36
 * Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// application includes
#include "CommandConsole.h"
#include "vPrintString.h"
#include "TaskSleep.h"
#include "ApplicationTasks.h"
#include "MachinePins.h"

///////////////////////////////////////////////////////////////////////////////
// HAL includes for RTSW board
#include "DeviceIOLib.h"
#include "ADCLib.h"
#include "DAC4921Lib.h"
#include "SPILib.h"
#include "LEDLib.h"
#include "SwitchLib.h"
#include "PortIOLib.h"
#include "QC7366Lib.h"
#include "InterruptLib.h"
#include "I2CLib.h"
#include "GyroFXASLib.h"
#include "StatusLED.h"

///////////////////////////////////////////////////////////////////////////////
// function prototypes
static void HartbeatTask(void *pvParameters);
static void StartHartbeatTask(void);

void vApplicationIdleHook( void );
void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);


///////////////////////////////////////////////////////////////////////////////
// file globals
static TaskHandle_t handle_HartbeatTask = NULL;

const uint8_t MotorDacChannel[N_MOTORS] = {0U, 1U, 2U};
const uint8_t MotorQcChannel[N_MOTORS] = {0U, 1U, 2U};

///////////////////////////////////////////////////////////////////////////////
// void StartHartbeatTask(void)
/*
 * Maakt de heartbeat-task aan voor de onboard status-LED.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; bewaart de task-handle als aanmaken lukt.
 */
static void StartHartbeatTask(void)
{
	BaseType_t result = pdFAIL;
	
	result = xTaskCreate(HartbeatTask, "tsk_Hartbeat", (configMINIMAL_STACK_SIZE), NULL, 0, &handle_HartbeatTask);
	if (result == pdPASS )
	{
	}
}

///////////////////////////////////////////////////////////////////////////////
/*
vApplication functies worden door de FreeRTOS kernel automatisch aangeroepen
bij de dergelijke situatie.
*/


///////////////////////////////////////////////////////////////////////////////
// void vApplicationIdleHook( void )
/*
 * FreeRTOS roept deze hook aan wanneer er geen andere task klaarstaat.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; momenteel wordt er niets uitgevoerd.
 */
void vApplicationIdleHook( void )
{
	//vPrintString("> idle task\n");
	//vTaskDelay((portTickType)(configTICK_RATE_HZ * 1.0));
}


///////////////////////////////////////////////////////////////////////////////
// void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
/*
 * FreeRTOS roept deze hook aan wanneer een task-stack overflow wordt gedetecteerd.
 * Invoer: xTask en pcTaskName verwijzen naar de task met de stackfout.
 * Uitvoer: geen returnwaarde; blijft snel met de status-LED knipperen als foutindicatie.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
	// Ga LED snel knipperen, fout indicatie
	while (true)
	{
		ioport_toggle_pin_level(PIN_STATUS_LED);
		delay_ms(50);
	}
}

///////////////////////////////////////////////////////////////////////////////
// void vApplicationMallocFailedHook(void)
/*
 * FreeRTOS roept deze hook aan wanneer dynamische geheugentoewijzing mislukt.
 * Invoer: geen.
 * Uitvoer: geen returnwaarde; blijft snel met de status-LED knipperen als foutindicatie.
 */
void vApplicationMallocFailedHook(void)
{
	// Ga LED snel knipperen, fout indicatie
	while (true)
	{
		ioport_toggle_pin_level(PIN_STATUS_LED);
		delay_ms(50);
	}
}


///////////////////////////////////////////////////////////////////////////////
// void HartbeatTask(void *pvParameters)
/*
 * Laat de onboard status-LED periodiek knipperen als alive-indicatie.
 * Invoer: pvParameters wordt niet gebruikt.
 * Uitvoer: geen returnwaarde; de taak blijft oneindig draaien.
 */
static void HartbeatTask(void *pvParameters)
{
	vPrintString("> Hartbeat should be running, flashing onboard LED...\n");
	
	// Laat LED op normale frequentie knipperen
	while (true)
	{
		// toggle led pin
		stled_Toggle();
		taskSleep(500);
		
		//led_DisplayValue(0x07);
	}
	
	/* Should never go here */
	vTaskDelete(NULL);
}


///////////////////////////////////////////////////////////////////////////////
// int main (void)
/*
 * Startpunt van de firmware.
 * Initialiseert klok, board, hardwaredrivers, taken en de FreeRTOS scheduler.
 * Invoer: geen.
 * Uitvoer: normaal geen returnwaarde; de scheduler neemt de uitvoering over.
 */
int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).
	sysclk_init();
	board_init();
	
	// Insert application code here, after the board has been initialized.
	// Ge
	delay_ms(200);

	// Deze functies initialiseren de hardware-drivers en communicatie protocollen.
	dio_Init();	// *** must be called first, inits all IO lines ***
	stled_Init();
	
	adc_Init();
	spi_Init();
	dac_Init(); 
	led_Init();
	switch_Init();
	port_Init();
	qc_Init();
	i2c_Init();

	// De functies aanmaken
	StartCommandConsoleTask(NULL);
	
	StartHartbeatTask();
	StartApplicationTasks();
	
	// De "ready-list" taken starten en plannen op prioriteit.
	vTaskStartScheduler();
	
	while (true)
	{
	}
	
	/* Should never get here */
}
