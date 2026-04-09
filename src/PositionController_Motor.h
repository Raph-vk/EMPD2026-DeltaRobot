/*
 * PositionController.h
 *
 * Created: 18-9-2023 10:52:38
 *  Author: rasmsmee
 */ 


#ifndef POSITIONCONTROLLER_H_
#define POSITIONCONTROLLER_H_

///////////////////////////////////////////////////////////////////////////////
// function prototypes

void posctrl_InitParameters(double wbmFactor);
void posctrl_RunController_MotorSide(void);

#endif /* POSITIONCONTROLLER_H_ */