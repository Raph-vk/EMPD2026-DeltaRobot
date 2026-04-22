/*
 * vPrintString.c
 *
 * Created: 11-9-2022 22:43:20
 *  Author: Roel Smeets
 */ 

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <asf.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>


///////////////////////////////////////////////////////////////////////////////
// application includes

#include "vPrintString.h"

///////////////////////////////////////////////////////////////////////////////
// #define's

#define MAX_PRINT_STRING_LENGTH	512

///////////////////////////////////////////////////////////////////////////////
// external objects

extern xQueueHandle printfQueue;
extern freertos_uart_if the_command_uart;

///////////////////////////////////////////////////////////////////////////////
// void vPrintString(const char *format, ...)

void vPrintString(const char *format, ...)
{
	va_list ap;
	char *p = NULL;
	int text_length = 0;

	taskENTER_CRITICAL();

	p = pvPortMalloc(MAX_PRINT_STRING_LENGTH);
	if (p == NULL)
	{
		taskEXIT_CRITICAL();
		return;
	}

	p[0] = '\0';
	
	// Add this line to show Taskname before printed text on console
	// Add #define INCLUDE_pcTaskGetTaskName		1 in freeRTOSConfig.h
	// sprintf(p,"[%s]: ", pcTaskGetTaskName(NULL));

	text_length = strlen(p);
	
	va_start(ap, format);
	vsnprintf(p + text_length, MAX_PRINT_STRING_LENGTH - text_length, format, ap);
	va_end(ap);

#if 1
	if(printfQueue != NULL)
	{
		// queue the POINTER to the text buffer if room available, 
		// free the buffer after printing in PrintfTask()
		if(xQueueSend(printfQueue, &p, 0) != pdPASS)
		{
			vPortFree(p);	// no room in queue, free text buffer
		}
	}
	else
	{
		vPortFree(p);
	}
#else
	portTickType max_block_time_ticks = 200UL / portTICK_RATE_MS;
	freertos_uart_write_packet(the_command_uart, p, text_length, max_block_time_ticks);
	vPortFree(p);
#endif

	taskEXIT_CRITICAL();
}
