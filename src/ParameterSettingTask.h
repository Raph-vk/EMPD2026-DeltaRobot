/*
 * ParameterSettingTask.h
 *
 * Created: 23-11-2023 13:20:37
 *  Author: rasmsmee
 */ 


#ifndef PARAMETERSETTINGTASK_H_
#define PARAMETERSETTINGTASK_H_

//////////////////////////////////////////////////////////////////////////////
// #define's

// Bandwidth limits

#define WBLFACTOR_MIN	0.100
#define WBLFACTOR_MAX	0.400

#define WBLTHRESHOLD	0.001

//////////////////////////////////////////////////////////////////////////////
// function prototypes

void ParameterSettingTask(void *pvParameters);


#endif /* PARAMETERSETTINGTASK_H_ */