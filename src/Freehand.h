/*
 * Freehand.h
 *
 * Zachte freehand-regeling voor handmatig positioneren.
 */

#ifndef FREEHAND_H_
#define FREEHAND_H_

#include <stdbool.h>

void FreeHand_Reset(void);
void FreeHand_Control(void);
bool FreeHand_PrintCurrentTcpPosition(void);

#endif /* FREEHAND_H_ */
