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
extern UINT T10MS_Counter;
extern BOOL T10MS_Flag;
extern UINT T50MS_Counter;
extern BOOL T50MS_Flag;

/** Protect buffer *********************************/
extern RxBuffer rxBuffer;
extern TxBuffer txBuffer;

#endif

