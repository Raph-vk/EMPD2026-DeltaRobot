/*
 * CommandConsole.h
 *
 * Created: 27-11-2015 15:28:11
 *  Author: Gerard Harkema
 */ 


#ifndef COMMAND_CONSOLE_H_
#define COMMAND_CONSOLE_H_

#include "vPrintString.h"

void StartCommandConsoleTask(void *pvParameters);
void CommandConsoleTask(void *pvParameters);
void PrintfTask(void *pvParameters);

#endif /* COMMAND_CONSOLE_H_ */