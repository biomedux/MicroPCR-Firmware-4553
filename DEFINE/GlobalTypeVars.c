/****************************************************
*	FileName 	:	GlobalTypeVars.c
*	Date	 	:	2012.08.11
*	Author		:	Yeon June
****************************************************/

/** Includes ***************************************/
#include "./DEFINE/GlobalTypeVars.h"

/** Define Extern Variables ********************/
UINT T10MS_Counter = 0x00;
BOOL T10MS_Flag = FALSE;
UINT T50MS_Counter = 0x00;
BOOL T50MS_Flag = FALSE;

RxBuffer rxBuffer;
TxBuffer txBuffer;