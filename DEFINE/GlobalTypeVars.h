/****************************************************
	FileName 	:	GlobalTypeVars.h
	Date	 	:	2012.08.11
	Author		:	Yeon June
****************************************************/

/** Duplicate check ********************************/
#ifndef __GLOBALTYPEVARS_H__
#define __GLOBALTYPEVARS_H__

/** Includes ***************************************/
#include "./DEFINE/GenericTypeDefs.h"
#include "./DEFINE/UserDefs.h"

/** For Timer Flags ********************************/
extern UINT T2MS_Counter;
extern BOOL T2MS_Flag;
extern UINT T30MS_Counter;
extern BOOL T30MS_Flag;

/** For pid control delay **************************/
// 150904 YJ
extern UINT freeRunningCounter;
extern BOOL freeRunning;
extern BYTE fallingTargetArrival;

// 150930 YJ For PWM Power Test
extern UINT powerCounter;

/** Protect buffer *********************************/
extern RxBuffer rxBuffer;
extern TxBuffer txBuffer;

#endif

