/***********************
* FileName : main.c
* Date     : 2015.07.15 Project rebuild for clean up

* Author   : Jun Yeon
***********************/

#ifndef __MAIN_C__
#define __MAIN_C__


#include "./CONFIG/BootLoader.h"	// for use interrupt & bootloader
#include "./CONFIG/Init.h"			// for user initialize

void main(void)
{
	Init();	// intialize all ports and vars(CONFIG/init.c)

	while(1)
	{
		MainLooper();	// usb routine & PCR routine(CONFIG/init.c)
	}
}


#endif	// end of __MAIN_C__



