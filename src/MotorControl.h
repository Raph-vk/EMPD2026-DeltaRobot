/*
 * MotorControl.h
 *
 * Created: 10-04-2026
 * Author : Raph van Koeveringe
 *
 * Doel van deze module
 * ---------------------
 * Deze module vormt de motor-hardwarelaag van de delta robot.
 *
 * In dit bestand staan alleen functies die direct met motoren, encoders,
 * DAC-uitgangen of home-switches te maken hebben. De homing-cyclus zelf staat
 * bewust niet in deze module, maar in Homing.c.
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

///////////////////////////////////////////////////////////////////////////////
// SYSTEM INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// function prototypes
bool motor_IsHomeLimitActive(uint8_t motorIndex);
bool anyHomeSwitchActive(void);

void ZetMotorSpanning(uint8_t motorIndex, float spanningVolt);
void ZetMotorSpanningen(float spanningVolt);

void Regelaar_INIT(void);
void removeRegelaarHistory(uint8_t arm);
float PIDregelaar(uint8_t motorIndex, float error);
float FeedForward(float alpha);



#endif /* MOTORCONTROL_H_ */
