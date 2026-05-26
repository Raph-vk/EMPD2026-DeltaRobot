
/*
 * Screen.h
 *
 * Created: 24/05/2026 16:50:27
 *  Author: raphv
 */
#ifndef SCREEN_H_
#define SCREEN_H_

///////////////////////////////////////////////////////////////////////////////
// system includes
#include <asf.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
// function prototypes

bool Screen_Init(void);
bool Screen_DrawStatus(const char *stateTxt,
					   const char *opTxt1,
					   const char *opTxt2,
					   const char *stroomTxt,
					   const char *potTxt);
bool Screen_Recover(void);
status_code_t Screen_GetLastTransferStatus(void);

#endif /* SCREEN_H_ */
