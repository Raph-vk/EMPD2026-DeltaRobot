/*
 * CommandConsole.c
 *
 * Created: 27-11-2015 15:27:46
 *  Author:		Gerard Harkema
 *  Adapted by:	Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <stdarg.h>

///////////////////////////////////////////////////////////////////////////////
// application includes

#include "CommandConsole.h"
#include "CLI-commands.h"
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "task.h"


///////////////////////////////////////////////////////////////////////////////
// #defines

#define STRING_EOL    "\r"
#define STRING_HEADER \
"-- FreeRTOS version: "tskKERNEL_VERSION_NUMBER"\r\n" \
"-- RASM Smeets\r\n" \
"-- Board type: "BOARD_NAME"\r\n" \
"-- Build: "__DATE__" "__TIME__" \n\n"

#define PRINTF_QUEUE_SIZE		10		// max. number of messages in queue
#define RECEIVE_BUFFER_SIZE		1024
#define MAX_INPUT_SIZE			1000


///////////////////////////////////////////////////////////////////////////////
// constant strings used by the CLI interface.

static const char *const welcome_message = 
		"FreeRTOS command server.\r\nType \"help\" to view a list of registered commands.\r\n\r\n";
static const char *const line_separator = 
		"\r\n[Press ENTER to execute the previous command again]\r\n";
static const char *const new_line = "\r\n";


///////////////////////////////////////////////////////////////////////////////
// file global data

freertos_uart_if the_command_uart = NULL;
char t_buffer[1000]; 	// non-reentrant!!
char *receive_buffer = NULL;

///////////////////////////////////////////////////////////////////////////////
// file global data: FreeRTOS task and queue handles

xTaskHandle handle_PrintfTask		  = NULL;
xTaskHandle handle_CommandConsoleTask = NULL;
xQueueHandle printfQueue			  = NULL;


freertos_peripheral_options_t  uart_driver_parameters = 
{
//	.interrupt_priority = 0x0e,
	.interrupt_priority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,	//of deze configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1,
	.options_flags = (USE_TX_ACCESS_SEM | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE),
	.receive_buffer = (uint8_t *) t_buffer,
	.receive_buffer_size = 1000,
	.operation_mode = UART_RS232
};


///////////////////////////////////////////////////////////////////////////////
// function prototypes
	

///////////////////////////////////////////////////////////////////////////////
// void StartCommandConsoleTask(void *pvParameters)

void StartCommandConsoleTask(void *pvParameters)
{
	char *p = NULL;
		
	printfQueue = xQueueCreate(PRINTF_QUEUE_SIZE, sizeof(p));
	
	xTaskCreate(PrintfTask, "tsk_Printf", 
				(configMINIMAL_STACK_SIZE * 2), 
				NULL, (configMAX_PRIORITIES - 2), &handle_PrintfTask);
				
	xTaskCreate(CommandConsoleTask, "tsk_CommandConsole", 
				(configMINIMAL_STACK_SIZE * 2), 
				NULL, (configMAX_PRIORITIES - 2), &handle_CommandConsoleTask);
}

	
///////////////////////////////////////////////////////////////////////////////
// static void ConfigureConsole(void)
	
static void ConfigureConsole(void)
{
	const sam_uart_opt_t uart_serial_options =
	{
		.ul_baudrate = CONF_UART_BAUDRATE,
		.ul_mck		 = sysclk_get_peripheral_hz(),
		.ul_mode	 = CONF_UART_PARITY
	};

	the_command_uart = freertos_uart_serial_init(
						CONF_UART, &uart_serial_options, &uart_driver_parameters);
	
	configASSERT(the_command_uart != NULL);
}


///////////////////////////////////////////////////////////////////////////////
// void printfTask(void *pvParameters)

void PrintfTask(void *pvParameters)
{
	char *p	= NULL;
	int text_length = 0;
	portTickType max_block_time_ticks = 200UL / portTICK_RATE_MS;

	while(true)
	{
		xQueueReceive(printfQueue, &p, portMAX_DELAY);
		text_length = strlen(p);
		freertos_uart_write_packet(the_command_uart, p,	text_length, max_block_time_ticks);
		vPortFree(p);
	}
	
	/* Should never go there */
	vTaskDelete(NULL);
}


///////////////////////////////////////////////////////////////////////////////
// void CommandConsoleTask(void *pvParameters)

void CommandConsoleTask(void *pvParameters)
{
	uint8_t received_char	= 0;
	uint8_t input_index		= 0;
	char *output_string		= NULL;
	static char input_string[MAX_INPUT_SIZE];
	static char last_input_string[MAX_INPUT_SIZE];
	portBASE_TYPE returned_value = 0;
	//portTickType max_block_time_ticks = 200UL / portTICK_RATE_MS;

	ConfigureConsole(); 
	
	/* Register the default CLI commands. */
	vRegisterCLICommands();

	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console
	interface will be used at any one time. */
	output_string = (char *) FreeRTOS_CLIGetOutputBuffer();

	/* Send the welcome message.  The message is copied into RAM first as the
	DMA cannot send from Flash addresses. */
	
	strcpy(output_string, STRING_HEADER);
	vPrintString(output_string);

	strcpy(output_string, welcome_message);
	vPrintString(output_string);

	while (true)
	{
		/* Only interested in reading one character at a time. */
		if (freertos_uart_serial_read_packet(the_command_uart, &received_char, sizeof(received_char), portMAX_DELAY) 
			== sizeof(received_char)) 
		{
			/* Echo the character. */
			// vPrintString("%c", received_char); 

			if (received_char == '\r') 
			{
				/* Start to transmit a line separator, just to make the output
				easier to read. */
				strcpy(output_string, new_line);
				vPrintString(output_string);

				/* See if the command is empty, indicating that the last command
				is to be executed again. */
				if (input_index == 0) 
				{
					strcpy(input_string, last_input_string);
				}

				/* Pass the received command to the command interpreter.  The
				command interpreter is called repeatedly until it returns pdFALSE as
				it might generate more than one string. */
				do 
				{
					/* Get the string to write to the UART from the command
					interpreter. */
					returned_value = FreeRTOS_CLIProcessCommand(
							input_string, output_string, configCOMMAND_INT_MAX_OUTPUT_SIZE);

					/* Start the UART transmitting the generated string. */
					vPrintString(output_string);
				} while (returned_value != pdFALSE);

				/* All the strings generated by the input command have been sent.
				Clear the input	string ready to receive the next command.
				Remember the command that was just processed first in case it is
				to be processed again. */
				
				strcpy(last_input_string, input_string);
				
				input_index = 0;
				memset(input_string, 0x00, MAX_INPUT_SIZE);

				/* Start to transmit a line separator, just to make the output
				easier to read. */
				strcpy(output_string, line_separator);
				vPrintString(output_string);
			} 
			else 
			{
				if (received_char == '\n') 
				{
					/* Ignore the character. */
				} 
				else if (received_char == '\b') 
				{
					/* Backspace was pressed.  Erase the last character in the
					string - if any. */
					if (input_index > 0)
					{
						input_index--;
						input_string[input_index] = '\0';
					}
				} 
				else 
				{
					/* A character was entered.  Add it to the string
					entered so far.  When a \n is entered the complete
					string will be passed to the command interpreter. */
					if (input_index < MAX_INPUT_SIZE) 
					{
						input_string[input_index] = received_char;
						input_index++;
					}
				}
			}
		}
	}
	/* Should never go there */
	vTaskDelete(NULL);
}
