/*
*  Map.h
*
*  Copyright (c) 2016 by Avans Hogeschool.
* 
*  Date:    28-may-2016
*  Author:  Roel Smeets
*/

#ifndef _MAP_H
#define _MAP_H


///////////////////////////////////////////////////////////////////////////////
// function prototypes

long int map(long int x, long int in_min,  long int in_max, 
			 			 long int out_min, long int out_max);

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
float constrain(float x, float min, float max);

#endif  // _MAP_H
