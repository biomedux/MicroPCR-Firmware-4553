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

// 160921 SCLee
UINT wg_ledCtrl = 0;
UINT r_ledCtrl = 0;
UINT g_ledCtrl = 0;
UINT b_ledCtrl = 0;
UINT wg_ledPwmDuty = 0;
UINT r_ledPwmDuty = 0;
UINT g_ledPwmDuty = 0;
UINT b_ledPwmDuty = 0;
UINT tmr0l_temp = 0;

RxBuffer rxBuffer;
TxBuffer txBuffer;

BYTE prevState = STATE_READY;
BYTE currentState = STATE_READY;