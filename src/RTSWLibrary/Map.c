/*
*  Map.c
*
*  Copyright (c) 2016 by Avans Hogeschool.
* 
*  Date:    16-may-2016
*  Author:  Roel Smeets
*
*/

///////////////////////////////////////////////////////////////////////////////
// system includes

#include <inttypes.h>
#include "Map.h"

///////////////////////////////////////////////////////////////////////////////
// long int map(long int x,
//			 long int in_min,  long int in_max, 
//			 long int out_min, long int out_max)
//
// Re-maps a number from one range to another.
//
// copied from Arduino source code library,
// see: https://www.arduino.cc/en/Reference/Map

long int map(long int x,
			 long int in_min,  long int in_max, 
			 long int out_min, long int out_max)
{
 	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


///////////////////////////////////////////////////////////////////////////////
// float fmap(float x,
//		   float in_min,  float in_max, 
//		   float out_min, float out_max)
//
// Re-maps a number from one range to another, floating point version
// of map

float fmap(float x,
		   float in_min,  float in_max, 
		   float out_min, float out_max)
{
 	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


///////////////////////////////////////////////////////////////////////////////
// float constrain(float x, float min, float max)
//
// limits / constrains a value x into the range [min..max]

float constrain(float x, float min, float max)
{
	float result = 0.0;

	if (x < min)
	{
		result = min;
	}
	else if (x > max)
	{
		result = max;
	}
	else
	{
		result = x;
	}

	return result;
}
