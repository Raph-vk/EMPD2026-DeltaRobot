/*
 *
 * \file
 *
 * \brief FreeRTOS+CLI command examples
 *
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "vPrintString.h"


/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#include "CLI-commands.h"

/*
 * Implements the run-time-stats command.
 */
static portBASE_TYPE task_stats_command(char *pcWriteBuffer,
		size_t xWriteBufferLen,
		const char *pcCommandString);

/*
 * Implements the task-stats command.
 */
static portBASE_TYPE run_time_stats_command(char *pcWriteBuffer,
		size_t xWriteBufferLen,
		const char *pcCommandString);



/*
 * Holds the handle of the task created by the create-task command.
 */
//static xTaskHandle created_task_handle = NULL;

/* Structure that defines the "run-time-stats" command line command.
This generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t run_time_stats_command_definition =
{
	"run-time-stats", /* The command string to type. */
	"run-time-stats: Displays a table showing how much processing time each FreeRTOS task has used\r\n",
	run_time_stats_command, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the "task-stats" command line command.  This generates
a table that gives information on each task in the system. */
static const CLI_Command_Definition_t task_stats_command_definition =
{
	"task-stats", /* The command string to type. */
	"task-stats: Displays a table showing the state of each FreeRTOS task\r\n",
	task_stats_command, /* The function to run. */
	0 /* No parameters are expected. */
};






/*-----------------------------------------------------------*/

void vRegisterCLICommands(void)
{
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand(&task_stats_command_definition);
	FreeRTOS_CLIRegisterCommand(&run_time_stats_command_definition);

}

/*-----------------------------------------------------------*/

static portBASE_TYPE task_stats_command(char *pcWriteBuffer,
		size_t xWriteBufferLen,
		const char *pcCommandString)
{
	const int8_t *const task_table_header = (int8_t *) "Task          State  Priority  Stack	#\r\n************************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Generate a table of task stats. */
	strcpy(pcWriteBuffer, (char *) task_table_header);
	vTaskList((pcWriteBuffer + strlen((char *) task_table_header)));

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE run_time_stats_command(char *pcWriteBuffer,
		size_t xWriteBufferLen,
		const char *pcCommandString)
{
	const int8_t *const stats_table_header = (int8_t *) "Task            Abs Time      % Time\r\n****************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Generate a table of task stats. */
	strcpy(pcWriteBuffer, (char *) stats_table_header);
	vTaskGetRunTimeStats((pcWriteBuffer + strlen((char *) stats_table_header)));

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}


/*-----------------------------------------------------------*/