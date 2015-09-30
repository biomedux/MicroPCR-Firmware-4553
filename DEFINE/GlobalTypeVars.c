/****************************************************
*	FileName 	:	GlobalTypeVars.c
*	Date	 	:	2012.08.11
*	Author		:	Yeon June
****************************************************/

/** Includes ***************************************/
#include "./DEFINE/GlobalTypeVars.h"

/** Define Extern Variables ********************/
UINT T2MS_Counter = 0x00;
BOOL T2MS_Flag = FALSE;
UINT T30MS_Counter = 0x00;
BOOL T30MS_Flag = FALSE;

/** For pid control delay **************************/
// 150904 YJ
UINT freeRunningCounter = 0x00;
BOOL freeRunning = FALSE;
BYTE fallingTargetArrival = 0;

// 150930 YJ
UINT powerCounter = 0;

RxBuffer rxBuffer;
TxBuffer txBuffer;