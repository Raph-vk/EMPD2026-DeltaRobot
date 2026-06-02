/*
 * Homing.h
 *
 * Created: 2026
 * Author : Raph van Koeveringe
 *
 * Doel van deze module
 * ---------------------
 * Deze module bevat de homing-cyclus van de drie delta-robot armen.
 *
 * homeAllMotors() is een niet-blokkerende step-functie. De functie moet vanuit
 * de 1 kHz control-loop worden aangeroepen totdat deze true teruggeeft.
 */

#ifndef HOMING_H_
#define HOMING_H_

///////////////////////////////////////////////////////////////////////////////
// SYSTEM INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// function prototypes
bool homeAllMotors(void);
void resetHoming(void);

#endif /* HOMING_H_ */
