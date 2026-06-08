
/*
 * MotionProfiles.h
 *
 * Created: 03/06/2026 21:10:31
 *  Author: raphv
 */ 
#ifndef MOTIONPROFILES_H_
#define MOTIONPROFILES_H_

#include <stdbool.h>

typedef struct
{
	float pos;   // incrementele positie
	float vel;   // snelheid
	float acc;   // acceleratie
	bool klaar;   // profiel-tijd is verstreken
} MotionProfileRef_t;

void motionProfile(float distance, float Tm, float elapsedTime_s, MotionProfileRef_t *ref);

#endif /* MOTIONPROFILES_H_ */
